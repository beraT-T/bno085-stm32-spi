/*
 * bno085.c — BNO085 için minimal SHTP-over-SPI sürücüsü (HAL).
 *
 * SPI mode 3 (CPOL=1, CPHA=2. kenar), MSB first, CS yazılımla (PD14).
 * Okuma akışı: INT low -> CS low -> 4 byte header -> kalan byte'lar -> CS high.
 * Yazma akışı: WAKE low -> INT low'u bekle -> WAKE high -> CS low -> paket -> CS high.
 *
 * SPI full-duplex: her clock'ta bir bit gider, bir bit gelir. Yani yazarken sensör
 * de kendi paketini gönderebilir; okurken de biz bir şey (sıfır) göndeririz.
 * Bu yüzden ikisi de aynı fonksiyondan geçer: okuma = "gönderecek paketi olmayan yazma".
 */
#include "bno085.h"
#include "timebase.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern SPI_HandleTypeDef hspi1;

#define SPI_TIMEOUT_MS   100U
#define SPI_CHUNK        32U
#define BNO085_MAX_PKT   2048U   /* bundan büyük uzunluk gelirse veriyi bozuk say */
#define WAKE_TIMEOUT_MS  10U     /* WAKE low'dan sonra INT'in low olması için süre */
#define TX_BUF_LEN       64U     /* en uzun komutumuz Set Feature: 4 + 17 byte */
#define PID_RECORD_LEN   16U     /* Product ID cevabındaki her kayıt */
#define FEATURE_LEN      17U     /* Set Feature komutu ve Get Feature cevabı */
#define CMD_REQ_LEN      12U     /* 0xF2 komut isteği */
#define CMD_RESP_LEN     16U     /* 0xF1 komut cevabı */
#define ME_CAL_CONFIGURE 0x00U   /* ME Calibration P3 alt komutu: ayarla */
#define ME_CAL_GET       0x01U   /* ME Calibration P3 alt komutu: oku */
#define FRS_WRITE_REQ_LEN 6U     /* 0xF7 FRS write isteği */
#define RAD2DEG          57.29578f
#define TS_TIE_US        200U    /* zaman damgası çözünürlüğü 100 µs (base ve delay ayrı ayrı
                                    yuvarlanır): bu kadar küçük geri adım "aynı an" sayılır */

/* ISR -> ana döngü haberleşmesi. volatile: derleyici her seferinde RAM'den okusun. */
static volatile bool     s_int_flag = false;
static volatile uint32_t s_int_cyc;      /* paketin İLK INT kenarının CYCCNT değeri (ISR yazar) */
static volatile bool     s_int_armed = true; /* sıradaki kenar yeni bir paketin ilk INT'i mi */
static uint32_t          s_pkt_int_cyc;  /* son transferin başında kilitlenen INT anı */
static bool              s_pkt_stale;    /* teşhis: kilitlerken yeni INT kenarı yoktu (eski referans) */
static uint32_t          s_pkt_lat_cyc;  /* teşhis: INT kenarından okumanın başına kadar geçen cycle */
static bno085_tback_t    s_tback;        /* teşhis: son "zaman geri gitti" olayı */
static uint32_t          s_stale_reads;  /* teşhis: eski referansla yapılan okuma sayısı */

/* Her kanalın kendi TX sequence sayacı. Sensör kayıp/tekrar paketi bununla
 * fark eder. uint8_t olduğu için 255'ten sonra kendiliğinden 0'a sarar. */
static uint8_t s_tx_seq[SHTP_NUM_CHANNELS];
static uint8_t s_tx_buf[TX_BUF_LEN];

/* SH-2 komut sıra numarası (0xF2'nin 1. byte'ı). SHTP header'ındaki seq'ten ayrı
 * bir sayaç: o paketleri sayar, bu komutları. */
static uint8_t s_cmd_seq;

/* Bizim gönderecek byte'ımız bitince MOSI'den giden byte'lar. Sıfır göndermek
 * önemli: sensör MOSI'yi de dinler ve sıfır header = "uzunluk 0" = "yazacak bir
 * şeyim yok" demektir. (HAL_SPI_Receive kullanılsaydı rx buffer'daki çöp gönderilirdi.) */
static const uint8_t s_zeros[SPI_CHUNK] = {0};
static uint8_t s_discard[SPI_CHUNK];

static inline void cs_low(void)    { HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_RESET); }
static inline void cs_high(void)   { HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET); }
static inline void wake_low(void)  { HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET); }
static inline void wake_high(void) { HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET); }

static inline bool int_is_low(void)
{
  return HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_RESET;
}

/* Little-endian 4 byte -> uint32 (SH-2'de tüm çok byte'lı alanlar LE). */
static inline uint32_t le32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Little-endian 2 byte -> int16 (sensör verileri işaretli). */
static inline int16_t le16s(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void put_le32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

/* Transferin [from, to) byte aralığını full-duplex yürütür.
 *   Giden:  i < tx_len ise tx[i], değilse 0.
 *   Gelen:  i < rx_cap ise rx[i]'ye yazılır, değilse atılır. Buffer'a sığmayan
 *           kısım yine de clock'lanır, yoksa sensör paketi bitmiş saymaz.
 * Parçalar SPI_CHUNK'ı ve tx/rx sınırlarını aşmayacak şekilde bölünür. */
static HAL_StatusTypeDef spi_xfer(const uint8_t *tx, uint16_t tx_len,
                                  uint8_t *rx, uint16_t rx_cap,
                                  uint16_t from, uint16_t to)
{
  uint16_t i = from;
  while (i < to)
  {
    uint16_t k = to - i;
    if (k > SPI_CHUNK)                   k = SPI_CHUNK;
    if (i < tx_len && k > tx_len - i)    k = tx_len - i;   /* tx biter -> sıfıra geç */
    if (i < rx_cap && k > rx_cap - i)    k = rx_cap - i;   /* rx dolar -> atmaya geç */

    const uint8_t *t = (i < tx_len) ? &tx[i] : s_zeros;
    uint8_t       *r = (i < rx_cap) ? &rx[i] : s_discard;
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)t, r, k, SPI_TIMEOUT_MS) != HAL_OK)
    {
      return HAL_ERROR;
    }
    i += k;
  }
  return HAL_OK;
}

/* CS low iken tek bir SHTP alışverişi (CS'i çağıran yönetir).
 *  1) İlk 4 byte: bizim header'ımız gider, aynı clock'larla sensörün header'ı gelir.
 *  2) Toplam uzunluk = max(giden, gelen). Böylece ne bizim paketimiz ne de
 *     sensörünki yarım kalır; kısa olan taraf bitince sıfır/çöp byte'lar akar.
 * Okuma için tx = NULL, tx_len = 0. *rx_len = gelen header'daki uzunluk. */
static bno085_status_t shtp_exchange(const uint8_t *tx, uint16_t tx_len,
                                     uint8_t *rx, uint16_t rx_cap, uint16_t *rx_len)
{
  /* 1) Header */
  if (spi_xfer(tx, tx_len, rx, rx_cap, 0U, SHTP_HEADER_LEN) != HAL_OK)
  {
    return BNO_ERR_SPI;
  }
  uint16_t len = (uint16_t)(rx[0] | (rx[1] << 8)) & 0x7FFFU;   /* bit15 = continuation */
  *rx_len = len;

  bno085_status_t st = BNO_OK;
  uint16_t in = len;                 /* sensörden okunacak toplam byte */
  if (len <= SHTP_HEADER_LEN)
  {
    in = 0U;                         /* 0 = sensörün gönderecek verisi yok */
  }
  else if (len > BNO085_MAX_PKT)
  {
    /* 0x7FFF gibi değerler genelde MISO'nun hep 1 okunduğu anlamına gelir.
     * Teşhis için toplam 16 byte oku: kaymış mı, rastgele mi görelim.
     * (Yazıyorsak paketimiz yine de tam gider, çünkü toplam = max(...).) */
    in = 16U;
    st = BNO_ERR_BAD_LEN;
  }

  /* 2) Kalan byte'lar, aynı CS penceresinde. */
  uint16_t total = (tx_len > in) ? tx_len : in;
  if (spi_xfer(tx, tx_len, rx, rx_cap, SHTP_HEADER_LEN, total) != HAL_OK)
  {
    return BNO_ERR_SPI;
  }
  return st;
}

void bno085_int_isr(void)
{
  /* Zaman referansı INT'in İLK düştüğü an (SHTP RM 2.6). Host geç kalırsa sensör INT'i
   * bırakıp yeniden düşürebilir; bu ikinci kenar AYNI paketindir ve zamanı ezmemeli.
   * Bu yüzden sadece "silahlıyken" yakala; okuma başlayınca (CS low) yeniden silahlanır. */
  if (s_int_armed)
  {
    s_int_cyc   = timebase_cycles();
    s_int_armed = false;
  }
  s_int_flag = true;
}

uint64_t bno085_last_int_us(void)
{
  return timebase_cycles_to_us(s_pkt_int_cyc);
}

bool bno085_data_ready(void)
{
  /* ISR bayrağı "falling edge geldi" der; asıl karar INT'in şu anki seviyesi.
   * INT, host paketi okuyana kadar low kalır, bu yüzden seviye güvenilirdir ve
   * edge kaçırılsa bile (ör. EXTI açılmadan önce düştüyse) veri kaybolmaz. */
  s_int_flag = false;
  return int_is_low();
}

bno085_status_t bno085_hard_reset(uint32_t timeout_ms)
{
  cs_high();
  /* PS0/WAKE = 1 (PS1 kartta 3V3'e bağlı) -> sensör reset çıkışında SPI modunu seçer. */
  wake_high();

  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);                       /* RST aktif low, ~10 ms tut */
  s_int_flag  = false;
  s_int_armed = true;
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);

  /* Reset sonrası sensörle ilk konuşma: seq sayaçları baştan başlasın. */
  memset(s_tx_seq, 0, sizeof(s_tx_seq));
  s_cmd_seq = 0;

  /* Sensör açılınca ilk paketi (advertisement) hazırlar ve INT'i low çeker. */
  uint32_t t0 = HAL_GetTick();
  while (!s_int_flag && !int_is_low())
  {
    if (HAL_GetTick() - t0 > timeout_ms)
    {
      return BNO_ERR_TIMEOUT;
    }
  }
  return BNO_OK;
}

bno085_status_t bno085_read_packet(uint8_t *buf, uint16_t buf_len, uint16_t *pkt_len)
{
  *pkt_len = 0;
  if (buf_len < SHTP_HEADER_LEN)
  {
    return BNO_ERR_PARAM;
  }
  if (!int_is_low())
  {
    return BNO_ERR_NO_DATA;            /* sensör hazır değilken transfer başlatma */
  }

  /* Bu paketin INT anını al, okumayı başlat, sonra bir sonraki paket için silahla:
     CS low'dan sonraki ilk düşen kenar artık yeni paketin ilk INT'idir. */
  s_pkt_stale   = s_int_armed;              /* hâlâ silahlı = son okumadan beri kenar yok */
  if (s_pkt_stale)
  {
    s_stale_reads++;
  }
  s_pkt_int_cyc = s_int_cyc;
  s_pkt_lat_cyc = timebase_cycles() - s_pkt_int_cyc;
  cs_low();
  s_int_armed = true;
  bno085_status_t st = shtp_exchange(NULL, 0U, buf, buf_len, pkt_len);
  cs_high();
  return st;
}

bno085_status_t bno085_write_packet(uint8_t channel, const uint8_t *payload, uint16_t payload_len,
                                    uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  *rx_pkt_len = 0;
  uint16_t tx_len = SHTP_HEADER_LEN + payload_len;
  if (channel >= SHTP_NUM_CHANNELS || tx_len > TX_BUF_LEN || rx_buf_len < SHTP_HEADER_LEN)
  {
    return BNO_ERR_PARAM;
  }

  /* 1) Paketi hazırla: 4 byte header + payload. */
  s_tx_buf[0] = (uint8_t)(tx_len & 0xFFU);   /* uzunluk LSB (header dahil) */
  s_tx_buf[1] = (uint8_t)(tx_len >> 8);      /* uzunluk MSB, bit15 continuation = 0 */
  s_tx_buf[2] = channel;
  s_tx_buf[3] = s_tx_seq[channel];
  if (payload_len > 0U)
  {
    memcpy(&s_tx_buf[SHTP_HEADER_LEN], payload, payload_len);
  }

  /* 2) Uyandır: WAKE (PS0) low -> sensör dinlemeye hazır olunca INT'i low çeker.
   *    INT zaten low ise (okunmayı bekleyen paketi var) beklemeden geçer. */
  wake_low();
  uint32_t t0 = HAL_GetTick();
  while (!int_is_low())
  {
    if (HAL_GetTick() - t0 > WAKE_TIMEOUT_MS)
    {
      wake_high();
      return BNO_ERR_TIMEOUT;
    }
  }
  wake_high();                               /* uyandı; PS0 normalde High kalmalı */

  /* 3) Transfer: paketimiz gider; sensörün o an hazır paketi varsa o da gelir. */
  s_pkt_stale   = s_int_armed;
  s_pkt_int_cyc = s_int_cyc;                /* yazarken gelen paket için de referans */
  s_pkt_lat_cyc = timebase_cycles() - s_pkt_int_cyc;
  cs_low();
  s_int_armed = true;
  bno085_status_t st = shtp_exchange(s_tx_buf, tx_len, rx, rx_buf_len, rx_pkt_len);
  cs_high();

  if (st != BNO_ERR_SPI)
  {
    s_tx_seq[channel]++;                     /* paket tam gitti -> sıradaki seq */
  }
  return st;
}

void bno085_dump_packet(const uint8_t *buf, uint16_t pkt_len)
{
  uint8_t ch   = buf[2];
  uint8_t seq  = buf[3];
  bool    cont = (buf[1] & 0x80U) != 0U;

  printf("[%6lu ms] ch=%u seq=%3u len=%4u%s | ",
         (unsigned long)HAL_GetTick(), ch, seq, pkt_len, cont ? " (cont)" : "");

  uint16_t payload = (pkt_len > SHTP_HEADER_LEN) ? (pkt_len - SHTP_HEADER_LEN) : 0U;
  uint16_t n = (payload > 16U) ? 16U : payload;
  for (uint16_t i = 0; i < n; i++)
  {
    printf("%02X ", buf[SHTP_HEADER_LEN + i]);
  }
  if (payload > n)
  {
    printf("...");
  }

  /* Tanıdığımız paketleri etiketle. */
  uint8_t first = (payload > 0U) ? buf[SHTP_HEADER_LEN] : 0xFFU;
  if (ch == SHTP_CH_COMMAND && first == 0x00U)                     printf(" <- advertisement");
  else if (ch == SHTP_CH_EXECUTABLE && first == 0x01U)             printf(" <- reset complete");
  else if (ch == SHTP_CH_CONTROL && first == SH2_COMMAND_RESP)     printf(" <- command response");
  else if (ch == SHTP_CH_CONTROL && first == SH2_PRODUCT_ID_RESP)  printf(" <- product ID response");
  else if (ch == SHTP_CH_CONTROL && first == SH2_GET_FEATURE_RESP) printf(" <- get feature response");
  else if (ch == SHTP_CH_CONTROL && first == SH2_FRS_WRITE_RESP)   printf(" <- FRS write response");
  printf("\r\n");
}

bool bno085_print_product_id(const uint8_t *buf, uint16_t pkt_len)
{
  if (pkt_len < SHTP_HEADER_LEN + PID_RECORD_LEN ||
      buf[2] != SHTP_CH_CONTROL || buf[SHTP_HEADER_LEN] != SH2_PRODUCT_ID_RESP)
  {
    return false;
  }

  /* SH-2 Reference Manual: reset sebebi kodları (sadece ilk kayıtta anlamlı). */
  static const char *const cause_str[] =
  {
    "yok", "power-on", "internal", "watchdog", "external (RST pini)", "diger"
  };

  /* Kayıt formatı (16 byte, çok byte'lı alanlar LE):
   *  [0] 0xF8  [1] reset sebebi  [2] SW major  [3] SW minor
   *  [4..7] SW part number  [8..11] SW build number  [12..13] SW patch  [14..15] reserved
   * Bir pakette birden fazla kayıt olabilir (her firmware bileşeni için bir tane). */
  uint16_t idx = 1;
  for (uint16_t off = SHTP_HEADER_LEN;
       off + PID_RECORD_LEN <= pkt_len && buf[off] == SH2_PRODUCT_ID_RESP;
       off += PID_RECORD_LEN, idx++)
  {
    const uint8_t *r = &buf[off];
    uint16_t patch   = (uint16_t)(r[12] | (r[13] << 8));

    printf("  Product ID #%u: SW v%u.%u.%u, part %lu, build %lu",
           idx, r[2], r[3], patch, (unsigned long)le32(&r[4]), (unsigned long)le32(&r[8]));
    if (r[1] != 0U)
    {
      printf(", reset sebebi %u (%s)", r[1], (r[1] <= 5U) ? cause_str[r[1]] : "?");
    }
    printf("\r\n");
  }
  return true;
}

bno085_status_t bno085_enable_report(uint8_t report_id, uint32_t interval_us,
                                     uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  /* Set Feature komutu (ch2, 17 byte, çok byte'lı alanlar LE):
   *  [0] 0xFD  [1] report ID  [2] flags = 0  [3..4] change sensitivity = 0
   *  [5..8] report interval (µs)  [9..12] batch interval = 0 (bekletmeden gönder)
   *  [13..16] sensor-specific config = 0 */
  uint8_t cmd[FEATURE_LEN] = {0};
  cmd[0] = SH2_SET_FEATURE_CMD;
  cmd[1] = report_id;
  put_le32(&cmd[5], interval_us);
  return bno085_write_packet(SHTP_CH_CONTROL, cmd, sizeof(cmd), rx, rx_buf_len, rx_pkt_len);
}

bool bno085_print_feature_resp(const uint8_t *buf, uint16_t pkt_len)
{
  if (pkt_len < SHTP_HEADER_LEN + FEATURE_LEN ||
      buf[2] != SHTP_CH_CONTROL || buf[SHTP_HEADER_LEN] != SH2_GET_FEATURE_RESP)
  {
    return false;
  }
  /* Format Set Feature ile aynı. Sensör istenen periyodu desteklemiyorsa en yakın
   * desteklediğini seçer: gerçek periyot buradaki değerdir. */
  const uint8_t *r = &buf[SHTP_HEADER_LEN];
  uint32_t interval = le32(&r[5]);
  if (interval == 0U)
  {
    printf("  Rapor 0x%02X kapali\r\n", r[1]);
  }
  else
  {
    printf("  Rapor 0x%02X acik: periyot %lu us (~%lu Hz)\r\n",
           r[1], (unsigned long)interval, (unsigned long)(1000000UL / interval));
  }
  return true;
}

/* SH-2 komut isteği (ch2, 12 byte):
 *  [0] 0xF2  [1] komut seq  [2] komut no  [3..11] P0..P8 (komuta göre anlamı değişir)
 * Cevap ch2'de 0xF1 olarak gelir (bkz. bno085_parse_command_resp). */
static bno085_status_t send_command(uint8_t cmd, const uint8_t p[9],
                                    uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  uint8_t req[CMD_REQ_LEN];
  req[0] = SH2_COMMAND_REQ;
  req[1] = s_cmd_seq++;
  req[2] = cmd;
  memcpy(&req[3], p, 9U);
  return bno085_write_packet(SHTP_CH_CONTROL, req, sizeof(req), rx, rx_buf_len, rx_pkt_len);
}

bno085_status_t bno085_get_cal_config(uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  uint8_t p[9] = {0};
  p[3] = ME_CAL_GET;                   /* sadece P3 anlamlı: "oku" */
  return send_command(SH2_CMD_ME_CAL, p, rx, rx_buf_len, rx_pkt_len);
}

bno085_status_t bno085_set_cal_config(uint8_t cal_flags,
                                      uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  /* P0 accel, P1 gyro, P2 mag, P3 alt komut, P4 planar accel, P5 on-table (1 = açık) */
  uint8_t p[9] = {0};
  p[0] = (cal_flags & BNO085_CAL_ACCEL)    ? 1U : 0U;
  p[1] = (cal_flags & BNO085_CAL_GYRO)     ? 1U : 0U;
  p[2] = (cal_flags & BNO085_CAL_MAG)      ? 1U : 0U;
  p[3] = ME_CAL_CONFIGURE;
  p[4] = (cal_flags & BNO085_CAL_PLANAR)   ? 1U : 0U;
  p[5] = (cal_flags & BNO085_CAL_ON_TABLE) ? 1U : 0U;
  return send_command(SH2_CMD_ME_CAL, p, rx, rx_buf_len, rx_pkt_len);
}

bool bno085_parse_command_resp(const uint8_t *buf, uint16_t pkt_len, bno085_cmd_resp_t *resp)
{
  if (pkt_len < SHTP_HEADER_LEN + CMD_RESP_LEN ||
      buf[2] != SHTP_CH_CONTROL || buf[SHTP_HEADER_LEN] != SH2_COMMAND_RESP)
  {
    return false;
  }
  /* [0] 0xF1  [1] seq  [2] komut no (bit7 = sensör kendiliğinden gönderdi)
   * [3] cevaplanan isteğin komut seq'i  [4] cevap seq  [5..15] R0..R10 */
  const uint8_t *r = &buf[SHTP_HEADER_LEN];
  resp->command     = r[2] & 0x7FU;
  resp->unsolicited = (r[2] & 0x80U) != 0U;
  memcpy(resp->r, &r[5], sizeof(resp->r));
  return true;
}

uint8_t bno085_cal_flags(const bno085_cmd_resp_t *resp)
{
  /* Get ME Calibration cevabı: R0 status, R1 accel, R2 gyro, R3 mag, R4 planar, R5 on-table */
  uint8_t f = 0;
  if (resp->r[1] != 0U) f |= BNO085_CAL_ACCEL;
  if (resp->r[2] != 0U) f |= BNO085_CAL_GYRO;
  if (resp->r[3] != 0U) f |= BNO085_CAL_MAG;
  if (resp->r[4] != 0U) f |= BNO085_CAL_PLANAR;
  if (resp->r[5] != 0U) f |= BNO085_CAL_ON_TABLE;
  return f;
}

bno085_status_t bno085_save_dcd(uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  uint8_t p[9] = {0};                  /* parametresi yok */
  return send_command(SH2_CMD_SAVE_DCD, p, rx, rx_buf_len, rx_pkt_len);
}

bno085_status_t bno085_set_dcd_autosave(bool enable,
                                        uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  uint8_t p[9] = {0};
  p[0] = enable ? 0x00U : 0x01U;       /* DİKKAT ters mantık: 0 = aç, 1 = kapat (RM 6.4.8) */
  return send_command(SH2_CMD_DCD_PERIODIC, p, rx, rx_buf_len, rx_pkt_len);
}

bno085_status_t bno085_soft_reset(uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len,
                                  uint32_t timeout_ms)
{
  /* Clear DCD and Reset: sensör RAM'deki DCD'yi siler ve hemen resetlenir. Normal bir
   * resette RAM'deki DCD flash'a yazılırdı; bu komut bunu atomik olarak engeller. */
  uint8_t p[9] = {0};
  bno085_status_t st = send_command(SH2_CMD_CLEAR_DCD_RESET, p, rx, rx_buf_len, rx_pkt_len);
  if (st != BNO_OK)
  {
    return st;                         /* WAKE'e cevap yok: sensör SPI'da değil ya da takılı */
  }
  memset(s_tx_seq, 0, sizeof(s_tx_seq));
  s_cmd_seq = 0;

  /* Komutun cevabı yok; resetin olduğunu, yeniden açılan sensörün ilk paketi olan ch0
   * advertisement'tan anlarız. Ondan önce okunanlar (reset öncesi kuyrukta kalan
   * raporlar) atılır. Sensör SPI modunda değilse okunan çöp hiç eşleşmez -> timeout. */
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < timeout_ms)
  {
    if (int_is_low() &&
        bno085_read_packet(rx, rx_buf_len, rx_pkt_len) == BNO_OK &&
        *rx_pkt_len > SHTP_HEADER_LEN &&
        rx[2] == SHTP_CH_COMMAND && rx[SHTP_HEADER_LEN] == 0x00U)
    {
      return BNO_OK;
    }
  }
  return BNO_ERR_TIMEOUT;
}

bno085_status_t bno085_frs_erase(uint16_t frs_type,
                                 uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len)
{
  /* FRS Write Request (ch2, 6 byte):
   *  [0] 0xF7  [1] reserved  [2..3] uzunluk (32 bit word, LE)  [4..5] FRS tipi (LE)
   * Uzunluk 0 -> "bu kaydı sil" (RM 6.3.3). Cevap: 0xF5 FRS Write Response. */
  uint8_t req[FRS_WRITE_REQ_LEN] = {0};
  req[0] = SH2_FRS_WRITE_REQ;
  req[4] = (uint8_t)(frs_type & 0xFFU);
  req[5] = (uint8_t)(frs_type >> 8);
  return bno085_write_packet(SHTP_CH_CONTROL, req, sizeof(req), rx, rx_buf_len, rx_pkt_len);
}

bool bno085_parse_frs_write_resp(const uint8_t *buf, uint16_t pkt_len, uint8_t *status)
{
  if (pkt_len < SHTP_HEADER_LEN + 4U ||
      buf[2] != SHTP_CH_CONTROL || buf[SHTP_HEADER_LEN] != SH2_FRS_WRITE_RESP)
  {
    return false;
  }
  *status = buf[SHTP_HEADER_LEN + 1U];
  return true;
}

/* Input report uzunlukları (SH-2 Reference Manual). Raporlarda uzunluk alanı yok,
 * bir pakette art arda birden fazla rapor olabilir: sonrakine geçmek için her
 * ID'nin boyunu bilmek zorundayız. 0 = bilinmiyor. */
static uint16_t report_len(uint8_t id)
{
  switch (id)
  {
    case SH2_BASE_TIMESTAMP:   return 5U;    /* [0xFB][base delta, 4 byte, 100 µs birimi] */
    case SH2_TIMESTAMP_REBASE: return 5U;    /* [0xFA][rebase delta, 4 byte] */
    case SH2_ACCELEROMETER:    return 10U;   /* 4 byte ortak baş + x, y, z */
    case SH2_GYROSCOPE_CAL:    return 10U;
    case SH2_MAGNETIC_FIELD_CAL: return 10U;
    case SH2_ROTATION_VECTOR:  return 14U;
    default:                   return 0U;
  }
}

/* x, y, z raporlarının (ivme, gyro, ...) ortak çözümü:
 * [4..5] x  [6..7] y  [8..9] z, int16 LE. Q noktası q ise değer = ham / 2^q. */
static void parse_vec3(const uint8_t *r, uint8_t q, bno085_vec3_t *v)
{
  float scale = 1.0f / (float)(1UL << q);
  v->x      = le16s(&r[4]) * scale;
  v->y      = le16s(&r[6]) * scale;
  v->z      = le16s(&r[8]) * scale;
  v->status = r[2] & 0x03U;
}

/* Bir raporun zaman/seq bilgisini günceller: kayıp sayımı + dt istatistiği. */
static void update_meta(bno085_meta_t *m, uint8_t id, uint8_t seq, uint64_t t_us,
                        uint64_t int_us, int32_t ref, uint16_t delay)
{
  if (m->seq_valid)
  {
    uint8_t gap = (uint8_t)(seq - m->seq);   /* uint8_t: 255 -> 0 sarmasını kendisi çözer */
    if (gap == 1U && t_us + TS_TIE_US < m->t_us)
    {
      m->t_back++;                           /* çözünürlükten büyük geri adım: olmamalı */
      s_tback.count++;
      s_tback.report_id   = id;
      s_tback.seq         = seq;
      s_tback.t_prev_us   = m->t_us;     s_tback.t_new_us   = t_us;
      s_tback.int_prev_us = m->int_us;   s_tback.int_new_us = int_us;
      s_tback.ref_prev    = m->ref;      s_tback.ref_new    = ref;
      s_tback.delay_prev  = m->delay;    s_tback.delay_new  = delay;
      s_tback.lat_new_us  = s_pkt_lat_cyc / (SystemCoreClock / 1000000U);
      s_tback.stale_new   = s_pkt_stale;
    }
    else if (gap == 1U)
    {
      /* Kayıpsız ardışık çift: aradaki süre gerçek örnekleme periyodudur. Çözünürlük
         içindeki küçük geri adım (sensör iki örneği neredeyse aynı anda verdi) dt = 0. */
      uint32_t dt = (t_us > m->t_us) ? (uint32_t)(t_us - m->t_us) : 0U;
      if (m->dt_n == 0U || dt < m->dt_min) m->dt_min = dt;
      if (m->dt_n == 0U || dt > m->dt_max) m->dt_max = dt;
      m->dt_sum += dt;
      m->dt_n++;
    }
    else if (gap != 0U)
    {
      m->lost += (uint32_t)(gap - 1U);       /* örn. 5'ten sonra 8 geldi: 6 ve 7 kayıp */
    }
  }
  m->seq       = seq;
  m->seq_valid = true;
  m->t_us      = t_us;
  m->int_us    = int_us;
  m->ref       = ref;
  m->delay     = delay;
  m->count++;
}

uint32_t bno085_stale_reads(void)
{
  return s_stale_reads;
}

const bno085_tback_t *bno085_tback_info(void)
{
  return &s_tback;
}

void bno085_data_restart(bno085_data_t *data)
{
  data->rv.meta.seq_valid    = false;
  data->accel.meta.seq_valid = false;
  data->gyro.meta.seq_valid  = false;
  data->mag.meta.seq_valid   = false;
}

bool bno085_parse_input_reports(const uint8_t *buf, uint16_t pkt_len, uint64_t t_int_us,
                                bno085_data_t *data)
{
  /* SH-2 zaman damgası (100 µs birim): örnek anı = INT anı + (ref + delay) * 100 µs
   *   0xFB base timestamp: ref = -base_delta  (base, INT'ten base_delta kadar ÖNCE)
   *   0xFA rebase:         ref += rebase_delta (uzun paketlerde referansı kaydırır)
   *   rapor delay'i:       ref'ten sonra geçen süre */
  int32_t ref = 0;

  uint16_t off = SHTP_HEADER_LEN;
  while (off < pkt_len)
  {
    const uint8_t *r = &buf[off];
    uint16_t n = report_len(r[0]);
    if (n == 0U || off + n > pkt_len)
    {
      return false;                  /* bilinmeyen ID ya da yarım rapor: kalanı atla */
    }

    if (r[0] == SH2_BASE_TIMESTAMP)
    {
      ref = -(int32_t)le32(&r[1]);
      off += n;
      continue;
    }
    if (r[0] == SH2_TIMESTAMP_REBASE)
    {
      ref += (int32_t)le32(&r[1]);
      off += n;
      continue;
    }

    /* Ortak rapor başı: [0] ID  [1] seq  [2] status  [3] delay LSB
     *   status bit1:0 = accuracy, bit7:2 = delay'in üst 6 biti -> delay 14 bit */
    uint16_t delay = (uint16_t)(((uint16_t)(r[2] & 0xFCU) << 6) | r[3]);
    uint64_t t_us  = (uint64_t)((int64_t)t_int_us + ((int64_t)ref + delay) * 100);

    if (r[0] == SH2_ROTATION_VECTOR)
    {
      /* [4..5] i  [6..7] j  [8..9] k  [10..11] real -> Q14: değer = ham / 2^14
       * [12..13] heading accuracy -> Q12 radyan: değer = ham / 2^12 */
      bno085_quat_t *q = &data->rv;
      q->i            = le16s(&r[4])  / 16384.0f;
      q->j            = le16s(&r[6])  / 16384.0f;
      q->k            = le16s(&r[8])  / 16384.0f;
      q->real         = le16s(&r[10]) / 16384.0f;
      q->accuracy_rad = le16s(&r[12]) / 4096.0f;
      q->status       = r[2] & 0x03U;
      update_meta(&q->meta, r[0], r[1], t_us, t_int_us, ref, delay);
    }
    else if (r[0] == SH2_ACCELEROMETER)
    {
      parse_vec3(r, 8U, &data->accel);   /* Q8 -> m/s² */
      update_meta(&data->accel.meta, r[0], r[1], t_us, t_int_us, ref, delay);
    }
    else if (r[0] == SH2_GYROSCOPE_CAL)
    {
      parse_vec3(r, 9U, &data->gyro);    /* Q9 -> rad/s */
      update_meta(&data->gyro.meta, r[0], r[1], t_us, t_int_us, ref, delay);
    }
    else if (r[0] == SH2_MAGNETIC_FIELD_CAL)
    {
      parse_vec3(r, 4U, &data->mag);     /* Q4 -> µT */
      update_meta(&data->mag.meta, r[0], r[1], t_us, t_int_us, ref, delay);
    }
    off += n;
  }
  return true;
}

void bno085_quat_to_euler(const bno085_quat_t *q, float *yaw, float *pitch, float *roll)
{
  float w = q->real, x = q->i, y = q->j, z = q->k;

  /* ZYX (önce yaw Z etrafında, sonra pitch Y, sonra roll X) dönüşüm formülleri.
   * asin'in argümanı Q14 yuvarlamasıyla 1'i az da olsa aşabilir -> NaN olmasın diye kırp. */
  float sinp = 2.0f * (w * y - z * x);
  if (sinp > 1.0f)
  {
    sinp = 1.0f;
  }
  else if (sinp < -1.0f)
  {
    sinp = -1.0f;
  }

  *roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD2DEG;
  *pitch = asinf(sinp) * RAD2DEG;
  *yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD2DEG;
}

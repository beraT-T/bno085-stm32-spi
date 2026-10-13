/*
 * bno085.h — BNO085 için minimal SHTP-over-SPI sürücüsü (HAL).
 *
 * SHTP paketi: [len LSB][len MSB | bit15 = continuation][channel][seq][payload...]
 * Uzunluk alanı 4 byte header'ı da içerir.
 */
#ifndef BNO085_H
#define BNO085_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define SHTP_HEADER_LEN      4U
#define BNO085_RX_BUF_LEN    512U    /* advertisement paketi ~280 byte, rahat sığar */

/* SHTP kanalları */
#define SHTP_CH_COMMAND      0U      /* SHTP komutları, advertisement */
#define SHTP_CH_EXECUTABLE   1U      /* reset complete vb. */
#define SHTP_CH_CONTROL      2U      /* sensor hub control (SH-2 komutları) */
#define SHTP_CH_REPORTS      3U      /* input report'lar */
#define SHTP_CH_WAKE_REPORTS 4U
#define SHTP_CH_GYRO_RV      5U
#define SHTP_NUM_CHANNELS    6U

/* SH-2 kontrol kanalı (ch2) rapor ID'leri */
#define SH2_PRODUCT_ID_REQ   0xF9U   /* istek:  [0xF9, 0x00(reserved)] */
#define SH2_PRODUCT_ID_RESP  0xF8U   /* cevap: 16 byte'lık kayıt(lar) */
#define SH2_SET_FEATURE_CMD  0xFDU   /* rapor aç/kapa, 17 byte */
#define SH2_GET_FEATURE_RESP 0xFCU   /* sensörün kabul ettiği ayar, 17 byte */
#define SH2_COMMAND_REQ      0xF2U   /* komut isteği: [0xF2][seq][komut][P0..P8], 12 byte */
#define SH2_COMMAND_RESP     0xF1U   /* komut cevabı: [0xF1][seq][komut][..][..][R0..R10], 16 byte */
#define SH2_FRS_WRITE_REQ    0xF7U   /* flash kaydı (FRS) yazma isteği; uzunluk 0 = kaydı sil */
#define SH2_FRS_WRITE_RESP   0xF5U   /* FRS yazma cevabı: [0xF5][status][offset LSB][offset MSB] */

/* FRS (flash record system) kayıt tipleri */
#define BNO085_FRS_DCD       0x1F1FU /* Dynamic Calibration Data: kaydedilmiş dinamik kalibrasyon */

/* SH-2 komut numaraları (0xF2 isteği / 0xF1 cevabı içinde) */
#define SH2_CMD_INITIALIZE   0x04U   /* reset sonrası sensör kendiliğinden de gönderir */
#define SH2_CMD_SAVE_DCD     0x06U   /* RAM'deki DCD'yi flash'a kaydet (cevap: R0 status) */
#define SH2_CMD_ME_CAL       0x07U   /* ME (motion engine) dinamik kalibrasyonu: ayarla / oku */
#define SH2_CMD_DCD_PERIODIC 0x09U   /* periyodik otomatik DCD kaydı aç/kapa (cevabı YOK) */
#define SH2_CMD_CLEAR_DCD_RESET 0x0BU /* RAM'deki DCD'yi sil + reset; flash'a yazmadan (cevabı YOK) */

/* Dinamik kalibrasyon bayrakları: hangi sensör çalışırken kendini kalibre etsin. */
#define BNO085_CAL_ACCEL     0x01U
#define BNO085_CAL_GYRO      0x02U
#define BNO085_CAL_MAG       0x04U
#define BNO085_CAL_PLANAR    0x08U   /* sadece düzlemde hareket eden cihazlar için */
#define BNO085_CAL_ON_TABLE  0x10U   /* masada duran cihazlar için */

/* Input report ID'leri (ch3) */
#define SH2_ACCELEROMETER    0x01U   /* m/s², Q8 (yerçekimi dahil) */
#define SH2_GYROSCOPE_CAL    0x02U   /* rad/s, Q9 (bias'ı sensör çıkarmış) */
#define SH2_MAGNETIC_FIELD_CAL 0x03U /* µT, Q4 (hard/soft iron kalibre edilmiş) */
#define SH2_ROTATION_VECTOR  0x05U   /* quaternion Q14 + heading accuracy Q12 (rad) */
#define SH2_TIMESTAMP_REBASE 0xFAU
#define SH2_BASE_TIMESTAMP   0xFBU

/* Her ölçümün "ne zaman / kaçıncı" bilgisi. */
typedef struct
{
  uint64_t t_us;          /* örneğin alındığı an, host µs saati (timebase): INT anı
                             + sensörün bildirdiği gecikmeler (base delta, delay) */
  uint8_t  seq;           /* raporun sıra numarası: sensör her raporda +1, 255 -> 0 */
  bool     seq_valid;     /* karşılaştırılacak önceki rapor var mı (reset sonrası false) */
  uint32_t count;         /* gelen rapor sayısı */
  uint32_t lost;          /* seq boşluklarından hesaplanan kayıp rapor sayısı (toplam) */
  uint32_t t_back;        /* ardışık örnekte zaman damgası 200 µs'den fazla geri gitti (olmamalı) */
  /* Teşhis: son örneğin zaman damgasının ham bileşenleri */
  uint64_t int_us;        /* paketinin INT referansı (host µs) */
  int32_t  ref;           /* -base (+rebase), 100 µs birim */
  uint16_t delay;         /* raporun delay alanı, 100 µs birim */
  /* Ardışık iki örnek arası süre (µs) istatistiği; okuyan taraf dt_n = dt_sum = 0
     yaparak yeni pencere başlatır. Sadece seq'i +1 olan (kayıpsız) çiftler sayılır. */
  uint32_t dt_min, dt_max;
  uint64_t dt_sum;
  uint32_t dt_n;
} bno085_meta_t;

/* Birim quaternion: w = real, (x, y, z) = (i, j, k). */
typedef struct
{
  float         i, j, k, real;
  float         accuracy_rad;   /* sensörün tahmini heading hatası (radyan) */
  uint8_t       status;         /* accuracy durumu: 0 = güvenilmez ... 3 = yüksek */
  bno085_meta_t meta;
} bno085_quat_t;

/* 3 eksenli ölçüm (ivme, gyro, ...), sensörün kendi eksenlerinde. */
typedef struct
{
  float         x, y, z;
  uint8_t       status;         /* accuracy durumu: 0 = güvenilmez ... 3 = yüksek */
  bno085_meta_t meta;
} bno085_vec3_t;

/* Teşhis: zaman damgası geri giden son olayın iki tarafı (önceki ve yeni örnek). */
typedef struct
{
  uint32_t count;                 /* toplam olay sayısı */
  uint8_t  report_id;
  uint8_t  seq;                   /* yeni örneğin seq'i (önceki = seq - 1) */
  uint64_t t_prev_us, t_new_us;   /* hesaplanan zaman damgaları */
  uint64_t int_prev_us, int_new_us; /* iki örneğin paketlerinin INT referansı */
  int32_t  ref_prev, ref_new;     /* -base (+rebase), 100 µs */
  uint16_t delay_prev, delay_new; /* delay alanı, 100 µs */
  uint32_t lat_new_us;            /* yeni paket: INT kenarından okumaya kadar geçen süre */
  bool     stale_new;             /* yeni paket okunurken yeni INT kenarı yakalanmamıştı */
} bno085_tback_t;

/* Input report'lardan çözülen son değerler (her birinin meta'sında zaman/seq). */
typedef struct
{
  bno085_quat_t rv;          /* 0x05 Rotation Vector */
  bno085_vec3_t accel;       /* 0x01 Accelerometer, m/s² */
  bno085_vec3_t gyro;        /* 0x02 Gyroscope calibrated, rad/s */
  bno085_vec3_t mag;         /* 0x03 Magnetic field calibrated, µT */
} bno085_data_t;

/* Çözülmüş SH-2 komut cevabı (0xF1). */
typedef struct
{
  uint8_t command;       /* hangi komutun cevabı (bit7 temizlenmiş) */
  bool    unsolicited;   /* sensör kendiliğinden gönderdi (bit7 = 1), istek yoktu */
  uint8_t r[11];         /* R0..R10; R0 neredeyse her komutta status (0 = başarılı) */
} bno085_cmd_resp_t;

typedef enum
{
  BNO_OK = 0,
  BNO_ERR_TIMEOUT,     /* INT beklenen sürede low olmadı */
  BNO_ERR_SPI,         /* HAL_SPI_* hata döndü */
  BNO_ERR_NO_DATA,     /* INT low değilken okuma istendi */
  BNO_ERR_BAD_LEN,     /* header'daki uzunluk anlamsız (MISO / SPI mode sorunu?) */
  BNO_ERR_PARAM,       /* geçersiz argüman (kanal no, paket çok uzun, buffer çok kısa) */
} bno085_status_t;

/* EXTI ISR'ından çağrılır: sadece bayrak set eder. */
void bno085_int_isr(void);

/* Sensörün okunmayı bekleyen verisi var mı? (INT pini low mu) */
bool bno085_data_ready(void);

/* RST pinini low-high yapıp sensörü yeniden başlatır, ilk INT'i bekler.
 * DİKKAT: power-up dışındaki resetlerde sensör RAM'deki DCD'yi flash'a yazar
 * (SH-2 RM 6.4.10). Mümkünse bno085_soft_reset kullan. */
bno085_status_t bno085_hard_reset(uint32_t timeout_ms);

/* Clear DCD and Reset (0x0B) komutuyla resetler: RAM'deki DCD silinir, flash'a
 * YAZILMADAN reset olur; açılışta flash'taki (en son bilerek kaydedilmiş) DCD yüklenir.
 * Sensör SPI'dan cevap vermelidir. Başarılıysa ilk paket (ch0 advertisement) rx'e
 * yazılır ve BNO_OK döner; timeout_ms içinde gelmezse BNO_ERR_TIMEOUT. */
bno085_status_t bno085_soft_reset(uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len,
                                  uint32_t timeout_ms);

/* Bir SHTP paketini okur. buf'a en fazla buf_len byte yazılır, fazlası okunup atılır.
 * *pkt_len = header'daki toplam uzunluk (continuation biti temizlenmiş). */
bno085_status_t bno085_read_packet(uint8_t *buf, uint16_t buf_len, uint16_t *pkt_len);

/* Bir SHTP paketi yazar: header'ı (uzunluk, kanal, kanalın seq sayacı) kendisi ekler.
 * SPI full-duplex olduğu için sensör aynı anda kendi paketini gönderebilir: o paket
 * rx'e yazılır ve *rx_pkt_len > SHTP_HEADER_LEN ise normal okunmuş paket gibi işlenmeli. */
bno085_status_t bno085_write_packet(uint8_t channel, const uint8_t *payload, uint16_t payload_len,
                                    uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len);

/* Debug: header'ı çözüp payload'ın ilk 16 byte'ını hex basar. */
void bno085_dump_packet(const uint8_t *buf, uint16_t pkt_len);

/* ch2 Product ID cevabıysa (0xF8) içindeki tüm kayıtları çözüp basar ve true döner.
 * Başka bir paketse hiçbir şey yapmaz, false döner. pkt_len buffer'a sığan kısım olmalı. */
bool bno085_print_product_id(const uint8_t *buf, uint16_t pkt_len);

/* Set Feature: report_id raporunu interval_us periyotla aç (0 = kapat).
 * Sensör ch2'de 0xFC (Get Feature Response) ile kabul ettiği ayarı bildirir.
 * rx / rx_len: bno085_write_packet'teki gibi, yazarken gelen paket. */
bno085_status_t bno085_enable_report(uint8_t report_id, uint32_t interval_us,
                                     uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len);

/* ch2 Get Feature Response (0xFC) ise çözüp basar ve true döner, değilse false. */
bool bno085_print_feature_resp(const uint8_t *buf, uint16_t pkt_len);

/* Son okunan/yazılan paketin zaman referansı: INT'in düştüğü an (host µs).
 * SPI'da SH-2 zaman damgaları bu ana göre verilir. */
uint64_t bno085_last_int_us(void);

/* ch3 paketindeki tüm input report'ları çözüp data'ya yazar. t_int_us: paketin
 * INT anı (bno085_last_int_us). Uzunluğunu bilmediğimiz bir rapor ID'si görürse
 * paketin kalanını atlar, false döner. pkt_len buffer'a sığan kısım olmalı. */
bool bno085_parse_input_reports(const uint8_t *buf, uint16_t pkt_len, uint64_t t_int_us,
                                bno085_data_t *data);

/* Teşhis: okuma başlarken bu pakete ait yeni bir INT kenarı yakalanmamıştı (sensör INT'i
 * bırakmadan sıradaki pakete geçmiş olabilir) -> referans eski, zaman damgası şüpheli. */
uint32_t bno085_stale_reads(void);

/* Teşhis: zaman damgası geri giden son olayın ayrıntıları (count değişince yeni olay). */
const bno085_tback_t *bno085_tback_info(void);

/* Sensör resetlenince çağır: seq'ler 0'dan yeniden başlar, eski seq ile
 * karşılaştırılıp sahte "kayıp" sayılmasın. Sayaçlar (count, lost) korunur. */
void bno085_data_restart(bno085_data_t *data);

/* ME Calibration ayarını sor (Get). Cevap ch2'de 0xF1 / komut 0x07 olarak gelir;
 * bno085_cal_flags() ile hangi kalibrasyonların açık olduğu çözülür. */
bno085_status_t bno085_get_cal_config(uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len);

/* ME Calibration ayarla (Configure): cal_flags = BNO085_CAL_* bayraklarının OR'u.
 * Ayar RAM'de tutulur; sensör resetlenince varsayılana döner. */
bno085_status_t bno085_set_cal_config(uint8_t cal_flags,
                                      uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len);

/* Save DCD (0x06): RAM'deki dinamik kalibrasyonu flash'a kaydet. Cevap 0xF1 / komut 0x06. */
bno085_status_t bno085_save_dcd(uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len);

/* Configure Periodic DCD Save (0x09): sensörün kendi kendine periyodik kaydını aç/kapa.
 * Bu komutun cevabı yok. */
bno085_status_t bno085_set_dcd_autosave(bool enable,
                                        uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len);

/* FRS kaydını flash'tan sil (FRS write, uzunluk 0). Cevap ch2'de 0xF5. */
bno085_status_t bno085_frs_erase(uint16_t frs_type,
                                 uint8_t *rx, uint16_t rx_buf_len, uint16_t *rx_pkt_len);

/* ch2 FRS Write Response (0xF5) ise *status'a yazar ve true döner.
 * status: 0 alındı, 1 bilinmeyen tip, 2 meşgul, 3 yazma tamam, 4 yazma moduna girildi,
 * 5 yazma hatası, 6 yazma modunda değil, 7 geçersiz uzunluk, 8 kayıt geçerli,
 * 9 kayıt geçersiz, 10 cihaz hatası, 11 kayıt salt okunur */
bool bno085_parse_frs_write_resp(const uint8_t *buf, uint16_t pkt_len, uint8_t *status);

/* ch2 Command Response (0xF1) ise *resp'i doldurur ve true döner, değilse false. */
bool bno085_parse_command_resp(const uint8_t *buf, uint16_t pkt_len, bno085_cmd_resp_t *resp);

/* Get ME Calibration cevabındaki R1..R5'i BNO085_CAL_* bayraklarına çevirir. */
uint8_t bno085_cal_flags(const bno085_cmd_resp_t *resp);

/* Quaternion -> yaw/pitch/roll (derece), ZYX sırası. pitch ±90°'de gimbal lock var. */
void bno085_quat_to_euler(const bno085_quat_t *q, float *yaw, float *pitch, float *roll);

#endif /* BNO085_H */

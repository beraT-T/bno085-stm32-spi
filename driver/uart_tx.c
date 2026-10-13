/*
 * uart_tx.c — LPUART1 için ring buffer + interrupt ile bloklamayan gönderim.
 *
 * Ring buffer (dairesel kuyruk), TX_BUF_SIZE hücre:
 *   head: ana döngünün bir sonraki byte'ı yazacağı yer  -> SADECE ana döngü değiştirir
 *   tail: UART'ın bir sonraki göndereceği byte          -> SADECE interrupt değiştirir
 *   head == tail     -> boş
 *   head + 1 == tail -> dolu (bir hücre hep boş kalır ki "dolu" ile "boş" ayırt edilsin)
 * Her değişkeni tek bir taraf yazdığı için interrupt kapatmaya (kilit) gerek yok.
 *
 * Gönderim: HAL_UART_Transmit_IT, tail'den başlayan bitişik parçayı (chunk) gönderir.
 * Parça bitince HAL interrupt içinden HAL_UART_TxCpltCallback'i çağırır; o da
 * tail'i ilerletir ve bu arada biriken byte varsa sıradaki parçayı başlatır.
 */
#include "uart_tx.h"
#include "main.h"
#include <stdbool.h>

extern UART_HandleTypeDef hlpuart1;

#define TX_BUF_SIZE  4096U                /* 2'nin kuvveti: index'i "& MASK" ile sarabilelim */
#define TX_BUF_MASK  (TX_BUF_SIZE - 1U)

static uint8_t           s_buf[TX_BUF_SIZE];
static volatile uint16_t s_head;          /* sonraki yazılacak hücre */
static volatile uint16_t s_tail;          /* sonraki gönderilecek hücre */
static volatile uint16_t s_chunk;         /* şu an gönderilen parçanın boyu, 0 = UART boşta */

static uint8_t           s_boot_log[UART_TX_BOOT_LOG];   /* açılış çıktısının kopyası */
static uint16_t          s_boot_len;
static bool              s_boot_capture = true;
static uint16_t          s_replay_pos   = UINT16_MAX;    /* tekrarın sıradaki byte'ı; MAX = kapalı */

/* tail'den başlayan bitişik bekleyen byte'ları göndermeye başlar.
 * Sadece UART boştayken (s_chunk == 0) çağrılır: ya ana döngüden, ya da bir
 * parça bitip callback s_chunk'ı sıfırladıktan sonra interrupt'tan. Boştayken
 * bitecek bir parça olmadığı için ikisi aynı anda çalışamaz. */
static void start_chunk(void)
{
  uint16_t head = s_head;
  uint16_t tail = s_tail;
  if (head == tail)
  {
    return;                               /* gönderilecek bir şey yok */
  }
  /* Bekleyen veri buffer'ın sonundan başa sarılıyorsa, önce sona kadar olan
     kısmı gönder; baştaki kısım bir sonraki callback'te gider. */
  uint16_t n = (head > tail) ? (uint16_t)(head - tail) : (uint16_t)(TX_BUF_SIZE - tail);

  s_chunk = n;                            /* Transmit_IT'den ÖNCE: callback erken gelebilir */
  if (HAL_UART_Transmit_IT(&hlpuart1, &s_buf[tail], n) != HAL_OK)
  {
    s_chunk = 0;                          /* başlamadı: bir sonraki putc'de tekrar denenir */
  }
}

void uart_tx_putc(uint8_t c)
{
  if (s_boot_capture && s_boot_len < UART_TX_BOOT_LOG)
  {
    s_boot_log[s_boot_len++] = c;
  }

  uint16_t next = (uint16_t)((s_head + 1U) & TX_BUF_MASK);

  /* Buffer dolu: interrupt byte gönderip tail'i ilerletene kadar bekle.
     Veri kaybetmek yerine kısa süre bloklamayı seçtik. Ortalama çıktımız UART
     kapasitesinin çok altında olduğu için normalde buraya hiç düşülmez. */
  while (next == s_tail)
  {
    if (s_chunk == 0U)
    {
      start_chunk();
    }
  }

  s_buf[s_head] = c;
  __DMB();                                /* önce byte belleğe yazılsın, SONRA head ilerlesin:
                                             interrupt yazılmamış bir hücreyi göndermesin */
  s_head = next;

  if (s_chunk == 0U)
  {
    start_chunk();                        /* UART boştaysa gönderimi başlat */
  }
}

void uart_tx_boot_log_replay(void)
{
  s_boot_capture = false;                 /* tekrar basarken kendini kaydetmesin */
  s_replay_pos   = 0;
}

bool uart_tx_replay_step(void)
{
  while (s_replay_pos < s_boot_len)
  {
    /* Kuyrukta en az 64 byte boş yer bırak: putc hiç beklemesin (ana döngü bloklanırsa
       sensörün çıkış kuyruğu taşar, rapor kaybolur ve zaman damgaları bozulabilir). */
    uint16_t used = (uint16_t)((s_head - s_tail) & TX_BUF_MASK);
    if (TX_BUF_SIZE - 1U - used < 64U)
    {
      return true;                        /* yer yok: sonraki turda devam */
    }
    uart_tx_putc(s_boot_log[s_replay_pos++]);
  }
  return false;
}

void uart_tx_boot_log_stop(void)
{
  s_boot_capture = false;
}

/* HAL, Transmit_IT ile başlatılan parça bitince bunu interrupt içinden çağırır. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &hlpuart1)
  {
    return;
  }
  s_tail  = (uint16_t)((s_tail + s_chunk) & TX_BUF_MASK);
  s_chunk = 0;
  start_chunk();                          /* bu arada biriken byte varsa devam */
}

/* printf -> _write (syscalls.c) -> __io_putchar -> kuyruğa ekle, hemen dön. */
int __io_putchar(int ch)
{
  uart_tx_putc((uint8_t)ch);
  return ch;
}

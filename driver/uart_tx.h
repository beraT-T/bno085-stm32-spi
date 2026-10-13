/*
 * uart_tx.h — LPUART1 (ST-LINK VCP) için interrupt'lı, bloklamayan gönderim.
 *
 * printf -> _write (syscalls.c) -> __io_putchar -> ring buffer -> LPUART1 interrupt
 *
 * printf karakterleri buffer'a yazıp hemen döner, byte'ları arka planda
 * HAL_UART_Transmit_IT gönderir. Böylece ana döngü UART'ı (115200 baud'da
 * byte başına ~87 µs) beklemez ve sensörü zamanında okuyabilir.
 */
#ifndef UART_TX_H
#define UART_TX_H

#include <stdint.h>
#include <stdbool.h>

/* Bir byte'ı gönderim kuyruğuna ekler. Buffer doluysa yer açılana kadar bekler.
 * Interrupt'lar kapalıyken çağırma: yer hiç açılmaz, sonsuza kadar bekler. */
void uart_tx_putc(uint8_t c);

/* Açılış logu: ilk UART_TX_BOOT_LOG byte'lık çıktının bir kopyası RAM'de de tutulur.
 * Kart USB'den güç alınca açıldığında ST-LINK'in seri portu MCU'dan ~1 s sonra hazır
 * olur; o ana kadar basılanlar bilgisayara ulaşmaz. Kopya sonradan tekrar basılabilir. */
#define UART_TX_BOOT_LOG 4096U

/* Kaydı durdurur ve saklanan açılış logunun tekrarını BAŞLATIR. Bloklamaz: gönderim
 * uart_tx_replay_step() ile, kuyrukta yer açıldıkça parça parça yapılır. */
void uart_tx_boot_log_replay(void);

/* Tekrarı ilerletir: kuyrukta yer varsa sıradaki byte'ları ekler. Ana döngü her turda
 * çağırır. Tekrar sürüyorsa true, bittiyse (ya da hiç başlamadıysa) false döner. */
bool uart_tx_replay_step(void);

/* Tekrar basmadan kaydı durdurur. */
void uart_tx_boot_log_stop(void);

#endif /* UART_TX_H */

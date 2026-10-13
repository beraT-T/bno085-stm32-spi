/*
 * timebase.h — DWT->CYCCNT tabanlı µs saati (ekstra timer gerekmez).
 *
 * CYCCNT, Cortex-M4'ün çekirdek saatiyle (80 MHz) artan 32 bit sayaç:
 * çözünürlük 12.5 ns, ama 2^32 / 80 MHz = 53.7 s'de taşar. Bu modül taşmaları
 * sayıp 64 bit'lik, açılıştan beri geçen µs saatine çevirir.
 *
 * Kural: timebase_update() en az 53 s'de bir çağrılmalı (ana döngü her turda
 * çağırıyor). 64 bit fonksiyonlar sadece ana döngüden çağrılır; ISR'da sadece
 * timebase_cycles() ile ham sayaç okunur, çevirme işi sonra ana döngüde yapılır.
 */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>
#include "main.h"

/* DWT cycle sayacını açar ve sıfırlar. HAL_Init + SystemClock_Config'ten sonra çağır. */
void timebase_init(void);

/* Ham 32 bit sayaç: ISR'da da güvenle okunabilir (tek register okuması). */
static inline uint32_t timebase_cycles(void)
{
  return DWT->CYCCNT;
}

/* Taşma sayımını günceller (bölme yok, ucuz). Ana döngü her turda çağırır. */
void timebase_update(void);

/* Açılıştan beri geçen µs (64 bit, ~584 bin yıl taşmaz). Sadece ana döngüden. */
uint64_t timebase_us(void);

/* Son ~53 s içinde timebase_cycles() ile alınmış bir örneği 64 bit µs'ye çevirir.
 * Örn. ISR'da yakalanan INT anı. Sadece ana döngüden. */
uint64_t timebase_cycles_to_us(uint32_t cyc);

#endif /* TIMEBASE_H */

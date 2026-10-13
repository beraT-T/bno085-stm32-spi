/*
 * timebase.c — DWT->CYCCNT'yi 64 bit µs saatine genişletir. Bkz. timebase.h.
 */
#include "timebase.h"

static uint32_t s_cyc_per_us;     /* 80 MHz -> 80 */
static uint32_t s_last_cyc;       /* son görülen ham CYCCNT */
static uint64_t s_total_cyc;      /* açılıştan beri toplam cycle (taşmalar dahil) */

void timebase_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* DWT/ITM bloğunu aç (debug'sız da çalışır) */
  DWT->CYCCNT       = 0;
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;       /* cycle sayacını başlat */

  s_cyc_per_us = SystemCoreClock / 1000000U;
  s_last_cyc   = 0;
  s_total_cyc  = 0;
}

/* Son çağrıdan beri geçen cycle'ı ekle. (uint32_t)(now - last) taşmayı kendiliğinden
   çözer: sayaç 0xFFFFFFF0'dan 0x10'a dönse bile fark 0x20 çıkar. Tek şart iki
   çağrı arasının 53.7 s'den kısa olması. */
static uint64_t extend(uint32_t now)
{
  s_total_cyc += (uint32_t)(now - s_last_cyc);
  s_last_cyc   = now;
  return s_total_cyc;
}

void timebase_update(void)
{
  (void)extend(DWT->CYCCNT);
}

uint64_t timebase_us(void)
{
  return extend(DWT->CYCCNT) / s_cyc_per_us;
}

uint64_t timebase_cycles_to_us(uint32_t cyc)
{
  uint32_t now   = DWT->CYCCNT;
  uint64_t total = extend(now);
  /* Örnek "şimdi"den (now - cyc) cycle önce alınmış: toplamdan o kadar geri git. */
  return (total - (uint32_t)(now - cyc)) / s_cyc_per_us;
}

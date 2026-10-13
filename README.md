# bno085-stm32-spi

A small, readable **SHTP-over-SPI driver for the CEVA BNO085 / BNO08x** on STM32 (HAL), with
per-sample microsecond timestamps, loss detection, calibration management and a browser-based
3D viewer.

It is written to be *understood*: every protocol step (SHTP framing, WAKE/INT handshake,
SH-2 commands, Q-point conversions, timestamp reconstruction) is implemented explicitly and
commented. If you need the full SH-2 feature set (FRS read, all sensors, DFU, sensor tare…),
use CEVA's official [`sh2`](https://github.com/ceva-dsp/sh2) library — this project was
cross-checked against it, but it is not a replacement for it.

> Code comments are in Turkish. Function names and this README are in English.

## Features

- **SPI transport (mode 3, ≤ 3 MHz)** with the BNO08x WAKE (PS0) / INT handshake.
  Writes are full duplex: a packet the sensor sends while we write is read completely, not truncated.
- **Reset & identification:** hard reset (RST pin), soft reset (*Clear DCD and Reset*),
  advertisement / reset-complete / initialize-response handling, Product ID decoding.
- **Reports:** Set Feature + Get Feature response (shows the rate the sensor *actually* accepted),
  parsing of Accelerometer (0x01), Gyroscope calibrated (0x02), Magnetic field calibrated (0x03)
  and Rotation Vector (0x05) with correct Q-points and accuracy status.
- **Timestamps:** INT falling edge captured with the DWT cycle counter (64-bit µs clock),
  combined with the SH-2 base-timestamp (0xFB), rebase (0xFA) and per-report delay fields —
  every sample gets the time it was actually measured.
- **Integrity:** per-report sequence-number tracking (lost samples), dt min/avg/max statistics,
  stale-INT and time-going-backwards diagnostics.
- **Calibration:** ME calibration get/configure (enables gyro dynamic calibration, which is off by
  default), Save DCD, periodic DCD autosave disable, FRS erase of the stored DCD.
- **Non-blocking printf:** LPUART1 ring buffer + interrupt, so logging never stalls sensor reads.
- **Tools:** Web Serial 3D viewer (Chrome/Edge, works offline) and a timestamping serial logger.

Tested on **NUCLEO-L4R5ZI-P** + GY-BNO08X breakout (BNO085, SH-2 firmware 3.2.13),
SPI 2.5 MHz: Rotation Vector / gyro / mag at 100 Hz and accelerometer at ~126 Hz with zero lost reports.

## Repository layout

```
driver/      bno085.c/.h   SHTP + SH-2 driver
             timebase.c/.h DWT-based 64-bit µs clock (ISR-safe capture)
             uart_tx.c/.h  non-blocking printf over LPUART1 (optional)
examples/nucleo-l4r5zi-p/  complete STM32CubeMX (CMake) project using the driver
tools/       bno085_viewer.html (+ three.min.js)  live 3D orientation viewer
             serial_log.py                         serial logger with timestamps
```

## Wiring (GY-BNO08X → NUCLEO-L4R5ZI-P)

| BNO08x pin | SPI role | Nucleo pin | Zio label | CubeMX |
|---|---|---|---|---|
| VCC / GND | supply | 3V3 / GND | | |
| SCL | SCK | PA5 | D13 | SPI1_SCK |
| SDA | MISO | PA6 | D12 | SPI1_MISO |
| ADD (SA0) | MOSI | PA7 | D11 | SPI1_MOSI |
| CS | chip select | PD14 | D10 | GPIO output, high, label `BNO_CS` |
| INT | host interrupt | PD15 | D9 | EXTI falling edge, pull-up, label `BNO_INT` |
| RST | reset | PF12 | D8 | GPIO output, high, label `BNO_RST` |
| PS0 | mode select + WAKE | PF13 | D7 | GPIO output, high, label `BNO_WAKE` |
| PS1 | mode select | 3V3 | | |
| BOOT | | not connected | | |

SPI mode is selected by **PS0 = PS1 = 1 at reset**. The firmware resets the sensor after the GPIOs
are configured, so PS0 is guaranteed high when the sensor samples it. On a custom PCB, add a
10 kΩ pull-up on PS0 so the sensor also boots in SPI mode before the MCU drives the pin.

## CubeMX settings (example project)

- SPI1: full-duplex master, software NSS, 8-bit, MSB first, **CPOL high / CPHA 2nd edge**,
  prescaler so that SCK ≤ 3 MHz (/32 at 80 MHz → 2.5 MHz).
- GPIO labels exactly as in the table above (the driver uses `BNO_CS_Pin`, `BNO_INT_Pin`, …).
  NVIC: EXTI line [15:10] enabled.
- LPUART1: 115200 8N1, global interrupt enabled (used by `uart_tx`).
- RCC: LSE crystal on, **MSI auto-calibration (PLL mode with LSE) enabled** — without it the MSI
  clock is off by ~0.5 % and every timestamp is scaled accordingly.

## Build & flash (example)

Requires `arm-none-eabi-gcc`, CMake ≥ 3.22 and Ninja (or the *STM32CubeIDE for VS Code* extension).

```bash
cd examples/nucleo-l4r5zi-p
cmake --preset Debug
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/nucleo_L4R5ZI_BNO085_spi.elf -v -rst
```

Open the ST-LINK virtual COM port at 115200 baud, or open `tools/bno085_viewer.html` in Chrome,
click *Connect* and rotate the sensor. The user button (B1) saves the calibration (hold 2–6 s and
release, only when accel/gyro/mag accuracy ≥ 2) or erases it (hold > 6 s).

## Porting to another STM32

1. Copy `driver/` into your project and add it to the build.
2. Provide a SPI handle named `hspi1` (mode 3, ≤ 3 MHz) and the four GPIO labels above.
3. Call `bno085_int_isr()` from `HAL_GPIO_EXTI_Callback` for the INT pin, and `timebase_init()` /
   `timebase_update()` at start-up / in the main loop.
4. On Cortex-M7 (e.g. STM32F7/H7) the DWT may be locked: write `DWT->LAR = 0xC5ACCE55` before
   enabling `CYCCNT`. At 216 MHz CYCCNT wraps every ~19.9 s, so `timebase_update()` must run more
   often than that.
5. `uart_tx` is optional; any `printf` backend works, but a blocking one will cost you reports.
   If you use it, it expects a UART handle named `hlpuart1` with its global interrupt enabled, and
   it defines `__io_putchar` and `HAL_UART_TxCpltCallback` (don't define them elsewhere).

Minimal read loop:

```c
static uint8_t rx[BNO085_RX_BUF_LEN];
static bno085_data_t imu;
uint16_t len;

if (bno085_data_ready() && bno085_read_packet(rx, sizeof rx, &len) == BNO_OK
    && len > SHTP_HEADER_LEN && rx[2] == SHTP_CH_REPORTS) {
    uint16_t valid = (len < sizeof rx) ? len : (uint16_t)sizeof rx;  /* part that fit in rx */
    bno085_parse_input_reports(rx, valid, bno085_last_int_us(), &imu);
    /* imu.rv (quaternion), imu.accel, imu.gyro, imu.mag — each with status and meta.t_us */
}
```

See `examples/nucleo-l4r5zi-p/Core/Src/main.c` for the full start-up sequence
(reset → Product ID → Set Feature → ME calibration) and the watchdog / re-enable logic.

## Things learned the hard way

- **SCK idle level:** with CPOL = 1 the SCK pin floats until the SPI peripheral is enabled. If SPE is
  set after CS goes low, the sensor sees an extra clock edge. Enable SPI once right after init.
- **Never block the main loop.** If reports are not read in time the sensor's output queue
  overflows: reports are dropped *and* the delay fields of the remaining ones can go backwards.
  Blocking UART output was enough to lose ~10 % of the reports.
- **Don't trust nominal rates.** The accelerometer accepted "8000 µs" but actually runs at
  ~7918 µs (~126 Hz). Use per-sample timestamps, and lock the MCU clock to a crystal first.
- **Gyro dynamic calibration is off by default** (ME calibration flags accel + mag only).
- Loose jumper wires produce exactly the symptoms of a protocol bug (INT timeouts, garbage
  lengths, random resets). Check wiring first.

## References

- CEVA: BNO08X Datasheet, SH-2 Reference Manual, SHTP Reference Manual
- CEVA `sh2` driver (Apache-2.0) — used for cross-checking, no code copied:
  https://github.com/ceva-dsp/sh2

## License

MIT — see [LICENSE](LICENSE).
Third-party files keep their own licenses: STM32 HAL / CMSIS under
`examples/nucleo-l4r5zi-p/Drivers` (STMicroelectronics / Arm), CubeMX-generated files
(STMicroelectronics), and `tools/three.min.js` (three.js, MIT).

---

### Türkçe özet

BNO085 için STM32 HAL tabanlı, okunabilir bir SPI (SHTP) sürücüsü: µs zaman damgası, kayıp
tespiti, kalibrasyon kaydetme/silme ve tarayıcıda canlı 3D görüntüleyici. NUCLEO-L4R5ZI-P üzerinde
test edildi. Kod yorumları Türkçe.

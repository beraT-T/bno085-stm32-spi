/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "bno085.h"
#include "timebase.h"
#include "uart_tx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Product ID isteğinin durumu (3. aşama) */
typedef enum
{
  PID_IDLE,      /* reset complete bekleniyor, istek henüz gitmedi */
  PID_WAITING,   /* istek gitti, 0xF8 cevabı bekleniyor */
  PID_DONE,      /* cevap geldi (ya da zaman aşımı uyarısı basıldı) */
} pid_state_t;

/* Bir raporun gerçek geliş hızını RATE_WINDOW_MS'lik pencerede ölçer. */
typedef struct
{
  uint32_t t0;   /* pencerenin başladığı an (0 = henüz başlamadı) */
  uint32_t n0;   /* o andaki rapor sayacı */
  uint32_t hz;   /* son ölçülen hız (0 = henüz ölçülmedi) */
} rate_t;

/* ME (dinamik) kalibrasyon ayarının durumu: oku -> gerekirse gyro'yu aç -> tekrar oku */
typedef enum
{
  CAL_IDLE,        /* raporlar açılınca CAL_SEND_GET'e geçer */
  CAL_SEND_GET,    /* ana döngü Get ME Calibration gönderecek */
  CAL_WAIT_GET,    /* okuma cevabı bekleniyor */
  CAL_SEND_SET,    /* gyro kapalı çıktı: ana döngü açan Configure'ı gönderecek */
  CAL_WAIT_SET,    /* ayarlama cevabı bekleniyor (sonra doğrulamak için tekrar Get) */
  CAL_SEND_DCD_OFF,/* sensörün kendi periyodik DCD kaydını kapat (cevabı yok) */
  CAL_DONE,
} cal_state_t;

/* Buton basılıyken LED geri bildirimi */
typedef enum
{
  LED_FB_OFF,      /* geri bildirim yok (LD1 normal heartbeat) */
  LED_FB_FAST,     /* 2 s geçti: bırakırsan KAYDET */
  LED_FB_SOLID,    /* 6 s geçti: bırakırsan SİL */
} led_fb_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_BLINK_MS    500U    /* LD1 (PC7) "yaşıyorum" yanıp sönmesi, yarım periyot */
#define LED_FAST_MS     100U    /* buton 2 s'yi geçti: hızlı yanıp sönme, yarım periyot */
#define FRS_TIMEOUT_MS  2000U   /* FRS silme (flash erase) için bekleme süresi */
#define BOOT_REPLAY_MS  4000U   /* güçle açılışta açılış logunu bu kadar sonra tekrar bas */

/* Buton geri bildirim LED'i: CubeMX'te PB14 (LD3, kırmızı) "LED_RED" etiketiyle
   GPIO_Output yapılmışsa o kullanılır (main.h'de LED_RED_Pin tanımlı olur);
   yapılmamışsa yeşil LD1 (heartbeat'i geçici olarak devralır). */
#ifdef LED_RED_Pin
#define FB_LED_PORT     LED_RED_GPIO_Port
#define FB_LED_PIN      LED_RED_Pin
#define FB_LED_IS_LD1   0
#else
#define FB_LED_PORT     Onboard_LED_GPIO_Port
#define FB_LED_PIN      Onboard_LED_Pin
#define FB_LED_IS_LD1   1
#endif
#define CMD_TIMEOUT_MS  1000U   /* SH-2 komut cevabı (Product ID, ME Calibration) için bekleme */
#define REPORT_INTERVAL_US 10000U  /* açılan tüm raporların periyodu: 10 ms = 100 Hz */
#define RAD2DEG         (180.0f / 3.14159265f)
#define PRINT_MS        100U    /* terminale basma periyodu: 10 Hz */
#define RATE_WINDOW_MS  1000U   /* gerçek rapor hızını bu pencerede ölç */
#define STALL_MS        2000U   /* bu kadar süre rapor gelmezse teşhis bas + sensörü resetle */
#define TIMING_MS       1000U   /* zaman damgası / kayıp özetini bu sıklıkla bas */
#define SOFT_RESET_MS   500U    /* Clear DCD and Reset sonrası advertisement bekleme süresi */
#define BTN_DEBOUNCE_MS 20U     /* buton bu kadar süre aynı kalırsa durumu kabul et */
#define BTN_SAVE_MS     2000U   /* 2..6 s basılı tutup bırak: DCD kaydet */
#define BTN_CLEAR_MS    6000U   /* >= 6 s basılı tutup bırak: kalibrasyonu sil */
#define SAVE_MIN_ACC    2U      /* kayıt için accel/gyro/mag accuracy en az bu olmalı (0..3) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static uint8_t bno_rx[BNO085_RX_BUF_LEN];   /* son okunan SHTP paketi */

static bool        bno_reset_done = false;  /* ch1 "reset complete" görüldü mü */
static pid_state_t pid_state      = PID_IDLE;
static uint32_t    pid_t0;                  /* isteğin gönderildiği an (ms) */

/* Reset sonrası sırayla açılacak raporlar (hepsi REPORT_INTERVAL_US periyotla). */
static const struct
{
  uint8_t     id;
  const char *name;
} reports[] =
{
  { SH2_ROTATION_VECTOR, "Rotation Vector" },
  { SH2_ACCELEROMETER,   "Accelerometer" },
  { SH2_GYROSCOPE_CAL,   "Gyroscope (calibrated)" },
  { SH2_MAGNETIC_FIELD_CAL, "Magnetic field (calibrated)" },  /* mag accuracy'si için */
};
#define NUM_REPORTS  (sizeof(reports) / sizeof(reports[0]))

static uint8_t       reports_sent = 0;      /* kaç Set Feature gönderildi (0 = hiç) */
static bno085_data_t imu;                   /* input report'lardan çözülen son değerler */

static uint32_t      last_alive;            /* son yeni rapor (ya da son reset) anı, ms */
static uint32_t      empty_reads;           /* INT low'du ama header uzunluğu 0 geldi */

static cal_state_t   cal_state     = CAL_IDLE;
static uint32_t      cal_t0;                /* son kalibrasyon komutunun gönderildiği an */
static uint8_t       cal_flags;             /* gönderilecek BNO085_CAL_* bayrakları */
static bool          cal_set_tried = false; /* Configure bir kez denendi mi (döngüye girmesin) */

static bool          dcd_save_pending = false; /* Save DCD gönderildi, cevabı bekleniyor */
static led_fb_t      led_fb = LED_FB_OFF;   /* button_step ayarlar, led_step uygular */
static bool          replay_active = false; /* açılış logu tekrarı sürüyor (canlı satırlar bekler) */
static uint32_t      dcd_save_t0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void print_clock_info(void);
static void bno_restart(void);
static void restart_state(void);
static bool wait_for_packet(uint8_t ch, uint8_t first, uint32_t timeout_ms);
static void button_step(void);
static void save_dcd(void);
static void clear_dcd(void);
static void led_step(void);
static void uart_cmd_step(void);
static void print_sec(uint64_t t_us);
static const char *frs_status_str(uint8_t st);
static void print_stall_diag(void);
static void handle_packet(uint16_t len);
static bool after_write(bno085_status_t st, uint16_t len, const char *what);
static void cal_config_step(void);
static void handle_command_resp(const bno085_cmd_resp_t *resp);
static void print_cal_flags(uint8_t f);
static void request_product_id(void);
static void enable_next_report(void);
static void rate_update(rate_t *r, uint32_t count, uint32_t now);
static void print_hz(const rate_t *r);
static void print_orientation(void);
static void print_timing(void);
static void print_meta(const char *name, bno085_meta_t *m);
static void print_fixed(const char *label, float v, uint8_t decimals, int width);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  timebase_init();                /* DWT cycle sayacı -> µs zaman damgaları */
  /* __DATE__/__TIME__: main.c'nin derlendiği an -> kartta hangi sürüm var, görürüz. */
  printf("\r\n=== BNO085 SPI test (build %s %s) ===\r\n", __DATE__, __TIME__);
  print_clock_info();

  /* MCU neden resetlendi? RCC_CSR bayrakları sadece güç tamamen kesilince sıfırlanır;
     okuyup temizliyoruz ki bir sonraki açılışta sadece o resetin sebebi görünsün.
     Her iç reset (yazılım, BOR, watchdog) NRST pininde de darbe ürettiği için PIN
     bayrağı neredeyse hep set olur: tek başına PIN = siyah RESET butonu. */
  bool power_on = __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST);
  bool rst_pin  = __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST);
  bool rst_sw   = __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST);
  bool rst_wdg  = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) || __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST);
  printf("MCU reset bayraklari:%s%s%s%s%s%s%s\r\n",
         power_on ? " BOR" : "", rst_pin ? " PIN" : "", rst_sw ? " SFT" : "",
         rst_wdg ? " WDG" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? " LPWR" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST)  ? " OBL"  : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_FWRST)   ? " FW"   : "");
  printf("  -> %s\r\n",
         power_on ? "guc verildi ya da besleme gerilimi dustu (BOR)" :
         rst_wdg  ? "watchdog reseti" :
         rst_sw   ? "yazilim reseti (ST-LINK ile flash/reset ya da NVIC_SystemReset)" :
         rst_pin  ? "siyah RESET butonu (NRST pini)" : "bilinmiyor");
  __HAL_RCC_CLEAR_RESET_FLAGS();
  printf("USER_BTN (PC13) bosta = %d (0 beklenir; 2 s basili: DCD kaydet, 6 s: kalibrasyonu sil)\r\n",
         (int)HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin));

  /* Sensörü temiz bir başlangıca al (önce soft reset, olmazsa RST pini; bkz. bno_restart). */
  bno_restart();

  uint32_t last_print    = HAL_GetTick();
  uint32_t last_timing   = HAL_GetTick();
  uint32_t printed_count = 0;     /* en son basılan RV raporunun sayaç değeri */
  uint32_t alive_count   = 0;     /* bekçinin en son gördüğü RV sayacı */
  bool     boot_log_done = false; /* açılış logu tekrarı yapıldı / gereksiz */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    timebase_update();              /* CYCCNT taşmalarını say (53 s'de bir taşar) */

    /* Sensörün verisi hazırsa (INT low) bir SHTP paketi oku ve hex bas. */
    if (bno085_data_ready())
    {
      uint16_t len;
      bno085_status_t st = bno085_read_packet(bno_rx, sizeof(bno_rx), &len);
      if (st == BNO_OK)
      {
        if (len <= SHTP_HEADER_LEN)
        {
          empty_reads++;    /* INT "verim var" dedi ama 0 uzunluk okundu: MISO/SCK/CS şüpheli */
        }
        handle_packet(len);
      }
      else
      {
        printf("Okuma hatasi %d, len=%u, ham: ", (int)st, len);
        for (int i = 0; i < 16; i++)
        {
          printf("%02X ", bno_rx[i]);
        }
        printf("\r\n");
        HAL_Delay(100);   /* hata durumunda terminali boğmamak için */
      }
    }

    /* 3. aşama: sensör "reset complete" dedikten sonra bir kez Product ID iste. */
    if (bno_reset_done && pid_state == PID_IDLE)
    {
      request_product_id();
    }
    if (pid_state == PID_WAITING && HAL_GetTick() - pid_t0 > CMD_TIMEOUT_MS)
    {
      printf("UYARI: %u ms icinde 0xF8 cevabi gelmedi.\r\n"
             "  -> Istek gitti mi (ustte HATA var mi)? MOSI kablosu (ADD pini -> PA7)?\r\n",
             CMD_TIMEOUT_MS);
      pid_state = PID_DONE;
    }

    /* 4-5. aşama: Product ID işi bitince raporları sırayla aç (döngü başına bir
       Set Feature; aradaki 0xFC cevapları normal okuma yolundan gelir). */
    if (pid_state == PID_DONE && reports_sent < NUM_REPORTS)
    {
      enable_next_report();
    }

    /* Raporlar açıldıktan sonra: dinamik kalibrasyon ayarını oku / gerekirse düzelt. */
    cal_config_step();

    /* User buton: 2 s -> DCD kaydet, 6 s -> kalibrasyonu sil. */
    button_step();

    /* Seri porttan gelen tek harfli test komutları (bkz. uart_cmd_step). */
    uart_cmd_step();

    /* Güç verilerek açıldıysak seri port o sırada henüz hazır değildi: açılış logu
       bilgisayara ulaşmadı. RAM'deki kopyasını bir kez tekrar bas (bloklamadan, aşağıda). */
    if (!boot_log_done && HAL_GetTick() >= BOOT_REPLAY_MS)
    {
      boot_log_done = true;
      if (power_on)
      {
        printf("\r\n===== ACILIS LOGU (guc verildiginde seri port hazir degildi, tekrar) =====\r\n");
        uart_tx_boot_log_replay();
        replay_active = true;
      }
      else
      {
        uart_tx_boot_log_stop();
      }
    }
    /* Tekrar bloklamadan, kuyrukta yer açıldıkça ilerler; sensör okuması hiç durmaz. */
    if (replay_active && !uart_tx_replay_step())
    {
      replay_active = false;
      printf("\r\n===== ACILIS LOGU SONU =====\r\n");
    }

    /* Sensör 100 Hz gönderir, biz 10 Hz basarız. Her raporu basmaya kalksak
       115200 baud'da UART yetişmez (bir satır ~10 ms sürer). */
    if (HAL_GetTick() - last_print >= PRINT_MS && !replay_active)
    {
      last_print = HAL_GetTick();   /* += değil: print uzarsa birikip patlamasın */
      if (imu.rv.meta.count != printed_count)   /* sadece yeni veri varsa bas, donmuş değeri değil */
      {
        printed_count = imu.rv.meta.count;
        print_orientation();
      }
    }

    /* Saniyede bir: her sensörün örnekleme aralığı (sensör zaman damgalarından) ve kayıplar. */
    if (HAL_GetTick() - last_timing >= TIMING_MS && !replay_active)
    {
      last_timing = HAL_GetTick();
      if (imu.rv.meta.count > 0U)
      {
        print_timing();
      }
    }

    /* Veri bekçisi: STALL_MS boyunca yeni rapor gelmezse sebebe dair ipucu bas,
       sensörü resetle ve açılış sırasını baştan başlat. Kablo düzelince kendini toparlar. */
    if (imu.rv.meta.count != alive_count)
    {
      alive_count = imu.rv.meta.count;
      last_alive  = HAL_GetTick();
    }
    else if (HAL_GetTick() - last_alive > STALL_MS)
    {
      print_stall_diag();
      bno_restart();
    }

    /* LED'ler: LD1 heartbeat + buton geri bildirimi (bloklamadan, SysTick'e bakarak). */
    led_step();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* HAL, SPI'ı (SPE biti) normalde ilk transferde açar. O ana kadar SCK pini
     sürülmez ve havada kalır. CS low olduktan sonra SPE açılırsa SCK low->high
     zıplar ve sensör bunu fazladan bir clock kenarı sayabilir -> bitler kayar.
     SPI'ı şimdi açınca SCK hemen CPOL=1 seviyesine (high) oturur. */
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, BNO_RST_Pin|BNO_WAKE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Onboard_LED_GPIO_Port, Onboard_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BNO_RST_Pin BNO_WAKE_Pin */
  GPIO_InitStruct.Pin = BNO_RST_Pin|BNO_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_CS_Pin */
  GPIO_InitStruct.Pin = BNO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BNO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_INT_Pin */
  GPIO_InitStruct.Pin = BNO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Onboard_LED_Pin */
  GPIO_InitStruct.Pin = Onboard_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Onboard_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* EXTI15_10 IRQ -> HAL_GPIO_EXTI_IRQHandler -> burası. ISR'da sadece bayrak. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BNO_INT_Pin)
  {
    bno085_int_isr();
  }
}

/* printf -> __io_putchar artık uart_tx.c'de: ring buffer + LPUART1 interrupt,
   printf UART'ı beklemeden döner. */

/* Saat ağacını register'lardan okuyup gerçek SPI1 SCK frekansını hesaplar.
   SPI1, APB2 (PCLK2) üzerinde. SPI_CR1.BR[2:0] alanı bölücüyü seçer:
   BR = 0..7  ->  fPCLK / 2^(BR+1)  (/2, /4, ... /256). */
static void print_clock_info(void)
{
  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();
  uint32_t br     = (SPI1->CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos;
  uint32_t div    = 2U << br;
  uint32_t sck    = pclk2 / div;

  printf("SYSCLK = %lu kHz, PCLK2 = %lu kHz\r\n",
         (unsigned long)(sysclk / 1000U), (unsigned long)(pclk2 / 1000U));
  printf("SPI1 SCK = PCLK2 / %lu = %lu kHz\r\n",
         (unsigned long)div, (unsigned long)(sck / 1000U));
  if (sck > 3000000U)
  {
    printf("UYARI: SCK > 3 MHz, BNO085 icin cok hizli! Prescaler'i artir.\r\n");
  }

  /* MSI (PLL'in kaynağı) tek başına ~%1 sapabilir. LSE (32.768 kHz kristal) açık ve
     MSIPLLEN=1 ise donanım MSI'ı sürekli kristale göre düzeltir. */
  bool lse    = (RCC->BDCR & RCC_BDCR_LSERDY) != 0U;
  bool msipll = (RCC->CR & RCC_CR_MSIPLLEN) != 0U;
  printf("Saat: LSE kristal %s, MSI otomatik kalibrasyon (LSE'ye kilit) %s\r\n",
         lse ? "CALISIYOR" : "kapali", msipll ? "ACIK" : "kapali");
}

/* Sensörü RST pininden resetler ve açılış sırasını (reset complete -> Product ID
   -> Rotation Vector) baştan başlatır. Açılışta ve veri bekçisi tetiklenince çağrılır.
   RST, sensör "takılmış" olsa bile (ör. besleme anlık kesilip pinlerden yarım
   beslendiyse) onu temiz bir başlangıca zorlar. */
static void bno_restart(void)
{
  uint16_t len;
  uint32_t t0 = HAL_GetTick();

  restart_state();

  /* 1) Önce soft reset (Clear DCD and Reset): sensör RAM'deki DCD'yi flash'a YAZMADAN
        yeniden başlar ve flash'taki (en son bilerek kaydettiğimiz) DCD'yi yükler.
        RST pini ise power-up dışı reset sayılır, sensör RAM'deki DCD'yi flash'a yazar
        (SH-2 RM 6.4.10) -> her açılışta/flash'lamada istemeden kayıt olurdu. */
  printf("BNO085 soft reset (Clear DCD and Reset)...\r\n");
  if (bno085_soft_reset(bno_rx, sizeof(bno_rx), &len, SOFT_RESET_MS) == BNO_OK)
  {
    printf("Sensor %lu ms'de yeniden basladi (RAM'deki DCD flash'a yazilmadi). Paketler:\r\n",
           (unsigned long)(HAL_GetTick() - t0));
    handle_packet(len);                    /* soft reset'in yakaladığı advertisement */
    last_alive = HAL_GetTick();
    return;
  }

  /* 2) Sensör SPI'dan cevap vermiyor (power-up'ta PS0 henüz ayarlı değilken yanlış
        modda açılmış olabilir, ya da takılmış): RST pini. PS0 artık kesin High,
        sensör reset çıkışında PS0=PS1=1 okuyup SPI moduna girer. */
  printf("  soft reset olmadi -> RST pini (hard reset)...\r\n");
  t0 = HAL_GetTick();
  if (bno085_hard_reset(500) == BNO_OK)
  {
    printf("INT low, %lu ms sonra. Paketler:\r\n", (unsigned long)(HAL_GetTick() - t0));
  }
  else
  {
    printf("HATA: reset sonrasi 500 ms icinde INT low olmadi!\r\n"
           "  -> VCC/GND, RST(PF12) High mi, PS0/PS1 ~3.3 V mu, INT kablosu (PD15)?\r\n");
  }
  last_alive = HAL_GetTick();          /* bekçi süresi resetten itibaren saysın */
}

/* Açılış sırasının (reset complete -> Product ID -> raporlar -> kalibrasyon) durumunu
   başa al. Sensör her resetlendiğinde (bizim ya da kendi isteğiyle) çağrılır. */
static void restart_state(void)
{
  bno_reset_done   = false;
  pid_state        = PID_IDLE;
  reports_sent     = 0;
  cal_state        = CAL_IDLE;             /* ME Calibration ayarı resetle varsayılana döner */
  cal_set_tried    = false;
  dcd_save_pending = false;
  bno085_data_restart(&imu);               /* sensörün seq'leri 0'dan başlayacak */
}

/* Kanalı ch ve ilk byte'ı first olan paket gelene kadar okur (bloklar, timeout'lu).
   Arada gelen paketler normal yoldan (handle_packet) işlenir. */
static bool wait_for_packet(uint8_t ch, uint8_t first, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < timeout_ms)
  {
    uint16_t len;
    if (bno085_data_ready() &&
        bno085_read_packet(bno_rx, sizeof(bno_rx), &len) == BNO_OK && len > SHTP_HEADER_LEN)
    {
      bool match = (bno_rx[2] == ch && bno_rx[SHTP_HEADER_LEN] == first);
      handle_packet(len);
      if (match)
      {
        return true;
      }
    }
  }
  return false;
}

/* User buton (PC13, Nucleo'da basılıyken high). Kazara işlem olmasın diye eylem
   bırakınca yapılır ve basılı tutma süresine göre seçilir:
     < 2 s  : hiçbir şey      2..6 s : DCD kaydet      >= 6 s : kalibrasyonu sil
   Kontaklar basarken/bırakırken birkaç ms zıplar (bounce): durum BTN_DEBOUNCE_MS
   boyunca değişmeden kalırsa kabul edilir. */
static void button_step(void)
{
  static bool     raw_last   = false;
  static uint32_t raw_change = 0;
  static bool     pressed    = false;     /* debounce edilmiş durum */
  static uint32_t t_down;
  static uint8_t  stage;                  /* 0: <2 s, 1: kaydet bölgesi, 2: sil bölgesi */

  uint32_t now = HAL_GetTick();
  bool     raw = (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_SET);
  if (raw != raw_last)
  {
    raw_last   = raw;
    raw_change = now;
  }

  if (now - raw_change >= BTN_DEBOUNCE_MS && raw != pressed)
  {
    pressed = raw;
    led_fb = LED_FB_OFF;                  /* basıldı ya da bırakıldı: geri bildirim baştan */
    if (pressed)
    {
      t_down = now;
      stage  = 0;
    }
    else if (stage == 1U)
    {
      save_dcd();
    }
    else if (stage == 2U)
    {
      clear_dcd();
    }
    else
    {
      printf("[buton] kisa basma: islem yok (2 s basili: kaydet, 6 s: sil)\r\n");
    }
  }
  else if (pressed)
  {
    uint32_t held = now - t_down;
    if (stage == 0U && held >= BTN_SAVE_MS)
    {
      stage  = 1;
      led_fb = LED_FB_FAST;
      printf("[buton] simdi birakirsan: DCD KAYDET  (6 s'ye kadar tutarsan: kalibrasyonu SIL)\r\n");
    }
    else if (stage == 1U && held >= BTN_CLEAR_MS)
    {
      stage  = 2;
      led_fb = LED_FB_SOLID;
      printf("[buton] simdi birakirsan: kalibrasyon SILINECEK (flash'taki DCD + RAM)\r\n");
    }
  }

  /* Save DCD cevabı handle_command_resp'te gelir; gelmezse uyar. */
  if (dcd_save_pending && now - dcd_save_t0 > CMD_TIMEOUT_MS)
  {
    printf("UYARI: %u ms icinde Save DCD cevabi gelmedi\r\n", CMD_TIMEOUT_MS);
    dcd_save_pending = false;
  }
}

/* Kalibrasyon yeterince iyiyse RAM'deki DCD'yi flash'a kaydettir. Kötü bir
   kalibrasyon kaydedilirse kalıcı olur ve flash'ın yazma ömrü sınırlıdır: bu yüzden
   sadece butonla ve accel/gyro/mag accuracy >= SAVE_MIN_ACC iken. */
static void save_dcd(void)
{
  uint8_t a = imu.accel.status, g = imu.gyro.status, m = imu.mag.status;
  uint16_t len;

  printf("[DCD] kaydet istegi: accel acc=%u gyro acc=%u mag acc=%u (en az %u gerekli)\r\n",
         a, g, m, SAVE_MIN_ACC);
  if (imu.mag.meta.count == 0U || a < SAVE_MIN_ACC || g < SAVE_MIN_ACC || m < SAVE_MIN_ACC)
  {
    printf("  -> KAYDEDILMEDI, sebep:");
    if (imu.mag.meta.count == 0U) printf(" mag raporu henuz gelmedi");
    if (a < SAVE_MIN_ACC)         printf(" accel acc=%u", a);
    if (g < SAVE_MIN_ACC)         printf(" gyro acc=%u", g);
    if (m < SAVE_MIN_ACC)         printf(" mag acc=%u", m);
    printf(" (< %u)\r\n", SAVE_MIN_ACC);
    printf("     Once kalibrasyon hareketleri (CEVA proseduru):\r\n"
           "     accel: 4-6 farkli yonde ~1 s sabit tut | gyro: 2-3 s masada birak |\r\n"
           "     mag: her eksende ~180 derece cevir ve geri (~2 s/eksen)\r\n");
    return;
  }

  printf("-> Save DCD (ch2: F2, komut 06)\r\n");
  dcd_save_pending = true;
  dcd_save_t0      = HAL_GetTick();
  bno085_status_t st = bno085_save_dcd(bno_rx, sizeof(bno_rx), &len);
  if (!after_write(st, len, "Save DCD"))
  {
    dcd_save_pending = false;
  }
}

/* Kaydedilmiş kalibrasyonu tamamen sil. SH-2 RM 6.4.10'un önerdiği sıra:
     1) hub'ı resetle  2) flash'taki DCD kaydını FRS ile sil  3) Clear DCD and Reset
   Sadece 3) yetmez: o, RAM'deki DCD'yi siler; flash'taki kopya açılışta yine yüklenir.
   Nadir, bilerek yapılan bir işlem olduğu için bloklayarak (~1 s) yapılıyor. */
static void clear_dcd(void)
{
  uint16_t len;

  printf("[DCD] kalibrasyon SILINIYOR\r\n");

  printf("  1/3 Clear DCD and Reset\r\n");
  restart_state();
  if (bno085_soft_reset(bno_rx, sizeof(bno_rx), &len, SOFT_RESET_MS) != BNO_OK ||
      !wait_for_packet(SHTP_CH_EXECUTABLE, 0x01U, CMD_TIMEOUT_MS))   /* reset complete */
  {
    printf("HATA: sensor soft reset'e cevap vermedi, silme yarida kesildi\r\n");
    bno_restart();
    return;
  }

  printf("  2/3 flash'taki DCD kaydi siliniyor (FRS write: 0xF7, tip 0x%04X, uzunluk 0)\r\n",
         BNO085_FRS_DCD);
  bno085_status_t st = bno085_frs_erase(BNO085_FRS_DCD, bno_rx, sizeof(bno_rx), &len);
  if (after_write(st, len, "FRS erase"))
  {
    /* Sensör birden fazla 0xF5 gönderir: önce 4 (yazma moduna girildi), iş bitince
       3 (yazma tamam). 0, 4, 8 ara durum -> beklemeye devam; 3 başarı; gerisi hata.
       3'ü beklemeden resetlersek silme yarıda kalabilir. */
    uint32_t t0      = HAL_GetTick();
    uint8_t  frs_st  = 0xFFU;
    bool     waiting = true;
    while (waiting && HAL_GetTick() - t0 < FRS_TIMEOUT_MS)
    {
      if (wait_for_packet(SHTP_CH_CONTROL, SH2_FRS_WRITE_RESP, FRS_TIMEOUT_MS) &&
          bno085_parse_frs_write_resp(bno_rx, sizeof(bno_rx), &frs_st))
      {
        waiting = (frs_st == 0U || frs_st == 4U || frs_st == 8U);
      }
    }
    if (frs_st == 3U)
    {
      printf("  -> flash'taki DCD kaydi SILINDI\r\n");
    }
    else
    {
      printf("UYARI: FRS silme bitmedi (son status %u: %s)\r\n", frs_st, frs_status_str(frs_st));
    }
  }

  printf("  3/3 Clear DCD and Reset\r\n");
  bno_restart();                           /* soft reset + normal açılış sırası */
}

/* LD1: 500 ms heartbeat ("firmware çalışıyor"). Buton basılıyken geri bildirim LED'i
   2 s'den sonra hızlı yanıp söner (bırakırsan KAYDET), 6 s'den sonra sürekli yanar
   (bırakırsan SİL). Geri bildirim LD1'deyse heartbeat o sırada durur. */
static void led_step(void)
{
  static uint32_t last_hb   = 0;
  static uint32_t last_fast = 0;
  uint32_t now = HAL_GetTick();

  if (!(FB_LED_IS_LD1 && led_fb != LED_FB_OFF) && now - last_hb >= LED_BLINK_MS)
  {
    last_hb = now;
    HAL_GPIO_TogglePin(Onboard_LED_GPIO_Port, Onboard_LED_Pin);
  }

  switch (led_fb)
  {
    case LED_FB_FAST:
      if (now - last_fast >= LED_FAST_MS)
      {
        last_fast = now;
        HAL_GPIO_TogglePin(FB_LED_PORT, FB_LED_PIN);
      }
      break;
    case LED_FB_SOLID:
      HAL_GPIO_WritePin(FB_LED_PORT, FB_LED_PIN, GPIO_PIN_SET);
      break;
    default:
      if (!FB_LED_IS_LD1)
      {
        HAL_GPIO_WritePin(FB_LED_PORT, FB_LED_PIN, GPIO_PIN_RESET);
      }
      break;
  }
}

/* Seri porttan (LPUART1 RX) tek harfli test komutları. Interrupt gerekmez: ana döngü
   RXNE bayrağına bakar. Bilgisayardan örn. `printf b > /dev/cu.usbmodemXXXX` ile gönderilir.
     'b' : ana döngüyü 300 ms blokla ("sensörü geç okuma" testi: zaman damgaları bozuluyor mu?)
     'r' : açılış logunu tekrar bas (bloklamadan) */
static void uart_cmd_step(void)
{
  if (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_OREFLAG(&hlpuart1);    /* okunmamış byte üstüne yenisi geldi: bayrağı temizle */
  }
  if (!__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE))
  {
    return;
  }
  uint8_t c = (uint8_t)(hlpuart1.Instance->RDR & 0xFFU);   /* RDR okumak RXNE'yi temizler */
  if (c == 'b')
  {
    printf("[test] ana dongu 300 ms bloklaniyor (gec okuma testi)\r\n");
    HAL_Delay(300);
  }
  else if (c == 'r' && !replay_active)
  {
    printf("\r\n===== ACILIS LOGU (istek uzerine tekrar) =====\r\n");
    uart_tx_boot_log_replay();
    replay_active = true;
  }
}

/* 64 bit µs'yi "saniye.mikrosaniye" bas (nano printf %llu desteklemiyor). */
static void print_sec(uint64_t t_us)
{
  printf("%lu.%06lu", (unsigned long)(t_us / 1000000U), (unsigned long)(t_us % 1000000U));
}

/* FRS Write Response status kodları (SH-2 RM 6.3.5). */
static const char *frs_status_str(uint8_t st)
{
  switch (st)
  {
    case 0:  return "word(ler) alindi";
    case 1:  return "bilinmeyen kayit tipi";
    case 2:  return "mesgul";
    case 3:  return "yazma tamamlandi";
    case 4:  return "yazma moduna girildi, bitmesi bekleniyor";
    case 5:  return "yazma hatasi";
    case 6:  return "yazma modunda degil";
    case 7:  return "gecersiz uzunluk";
    case 8:  return "kayit gecerli";
    case 9:  return "kayit gecersiz";
    case 10: return "cihaz hatasi";
    case 11: return "kayit salt okunur";
    default: return "cevap yok / bilinmiyor";
  }
}

/* Veri kesilince: o anki INT seviyesine ve boş okuma sayısına bakarak hangi
   hattın şüpheli olduğunu söyler. */
static void print_stall_diag(void)
{
  bool int_low = bno085_data_ready();  /* INT pininin şu anki seviyesi */

  printf("\r\nUYARI: %lu ms'dir yeni rapor yok. INT=%s, bos okuma=%lu, son header: %02X %02X %02X %02X\r\n",
         (unsigned long)(HAL_GetTick() - last_alive), int_low ? "LOW" : "HIGH",
         (unsigned long)empty_reads, bno_rx[0], bno_rx[1], bno_rx[2], bno_rx[3]);
  if (int_low || empty_reads > 0U)
  {
    /* Sensör "verim var" diyor, biz okuyoruz ama hep 0 geliyor. */
    printf("  -> Sensor veri vermek istiyor ama okunamiyor: MISO (SDA->PA6), SCK (SCL->PA5), CS (PD14)?\r\n");
  }
  else
  {
    /* Sensör hiç "verim var" demiyor. */
    printf("  -> INT hic low olmuyor: INT kablosu (PD15), sensor beslemesi (VCC/GND) ya da sensor takildi.\r\n");
  }
  empty_reads = 0;
}

/* bno_rx'teki bir SHTP paketini işler: hex dump + tanıdığımız paketlere tepki.
   Normal okumadan da, yazma sırasında (full-duplex) gelen paket de buradan geçer. */
static void handle_packet(uint16_t len)
{
  if (len <= SHTP_HEADER_LEN)
  {
    return;                                   /* boş transfer: sensörün verisi yoktu */
  }
  /* Paket buffer'dan uzunsa çözücülere sadece sığan kısmı ver. */
  uint16_t valid = (len < sizeof(bno_rx)) ? len : (uint16_t)sizeof(bno_rx);
  uint8_t  ch    = bno_rx[2];
  uint8_t  first = bno_rx[SHTP_HEADER_LEN];   /* payload'ın ilk byte'ı = rapor/komut ID */

  /* Input report'lar 100 Hz gelir: hex basmak UART'ı boğar, sadece çöz. */
  if (ch == SHTP_CH_REPORTS)
  {
    static bool warned = false;
    if (!bno085_parse_input_reports(bno_rx, valid, bno085_last_int_us(), &imu) && !warned)
    {
      printf("UYARI: ch3'te taninmayan rapor ID (sadece ilk sefer basiliyor):\r\n");
      bno085_dump_packet(bno_rx, len);
      warned = true;
    }
    return;
  }

  bno085_dump_packet(bno_rx, len);

  if (ch == SHTP_CH_EXECUTABLE && first == 0x01U)
  {
    if (reports_sent > 0U)
    {
      /* Biz resetlemedik ama reset complete geldi: sensör kendi kendine resetlendi
         (besleme teması koptu-geldi?). Reset sonrası tüm raporlar kapalı -> yeniden aç. */
      printf("UYARI: sensor kendiliginden resetlendi (VCC/GND temasi?) -> raporlar yeniden aciliyor\r\n");
      reports_sent  = 0;
      cal_state     = CAL_IDLE;
      bno085_data_restart(&imu);
      cal_set_tried = false;
    }
    bno_reset_done = true;
  }
  if (bno085_print_product_id(bno_rx, valid) && pid_state == PID_WAITING)
  {
    /* Cevap birden fazla pakete bölünebiliyor: süreyi sadece ilkinde bas. */
    printf("  (istekten %lu ms sonra)\r\n", (unsigned long)(HAL_GetTick() - pid_t0));
    pid_state = PID_DONE;
  }
  (void)bno085_print_feature_resp(bno_rx, valid);

  bno085_cmd_resp_t resp;
  if (bno085_parse_command_resp(bno_rx, valid, &resp))
  {
    handle_command_resp(&resp);
  }

  uint8_t frs_status;
  if (bno085_parse_frs_write_resp(bno_rx, valid, &frs_status))
  {
    printf("  FRS yazma cevabi: status %u (%s)\r\n", frs_status, frs_status_str(frs_status));
  }
}

/* ME Calibration akışının ana döngü tarafı: komut gönderir, zaman aşımına bakar.
   Cevaplar handle_command_resp()'ta işlenip durumu ilerletir. */
static void cal_config_step(void)
{
  uint16_t        len;
  bno085_status_t st;

  switch (cal_state)
  {
    case CAL_IDLE:
      if (reports_sent == NUM_REPORTS)
      {
        cal_state = CAL_SEND_GET;
      }
      break;

    case CAL_SEND_GET:
      printf("-> ME kalibrasyon ayari okunuyor (ch2: F2, komut 07, Get)\r\n");
      cal_t0    = HAL_GetTick();
      cal_state = CAL_WAIT_GET;
      st = bno085_get_cal_config(bno_rx, sizeof(bno_rx), &len);
      if (!after_write(st, len, "Get ME Calibration"))
      {
        cal_state = CAL_SEND_DCD_OFF;
      }
      break;

    case CAL_SEND_SET:
      printf("-> ME kalibrasyon ayarlaniyor (ch2: F2, komut 07, Configure):");
      print_cal_flags(cal_flags);
      cal_set_tried = true;
      cal_t0        = HAL_GetTick();
      cal_state     = CAL_WAIT_SET;
      st = bno085_set_cal_config(cal_flags, bno_rx, sizeof(bno_rx), &len);
      if (!after_write(st, len, "Configure ME Calibration"))
      {
        cal_state = CAL_SEND_DCD_OFF;
      }
      break;

    case CAL_SEND_DCD_OFF:
      /* Sensör DCD'yi kendi kendine periyodik kaydetmesin: kayıt sadece bizim
         kontrolümüzde (buton). Bu komutun cevabı yok. */
      printf("-> Periyodik DCD kaydi kapatiliyor (ch2: F2, komut 09, P0=1; cevabi yok)\r\n");
      cal_state = CAL_DONE;
      st = bno085_set_dcd_autosave(false, bno_rx, sizeof(bno_rx), &len);
      (void)after_write(st, len, "Configure Periodic DCD Save");
      break;

    case CAL_WAIT_GET:
    case CAL_WAIT_SET:
      if (HAL_GetTick() - cal_t0 > CMD_TIMEOUT_MS)
      {
        printf("UYARI: %u ms icinde ME kalibrasyon cevabi (0xF1, komut 07) gelmedi\r\n",
               CMD_TIMEOUT_MS);
        cal_state = CAL_SEND_DCD_OFF;
      }
      break;

    default:
      break;
  }
}

/* ch2 komut cevabı (0xF1) geldi: neyin cevabı olduğuna göre bas ve akışı ilerlet. */
static void handle_command_resp(const bno085_cmd_resp_t *resp)
{
  uint8_t status = resp->r[0];

  if (resp->command == SH2_CMD_INITIALIZE)
  {
    printf("  Initialize cevabi: status %u%s\r\n", status,
           resp->unsolicited ? " (reset sonrasi sensor kendiliginden gonderdi)" : "");
    return;
  }
  if (resp->command == SH2_CMD_SAVE_DCD)
  {
    printf("  Save DCD cevabi: status %u -> %s\r\n", status,
           (status == 0U) ? "kalibrasyon flash'a KAYDEDILDI" : "HATA, kaydedilemedi");
    dcd_save_pending = false;
    return;
  }
  if (resp->command != SH2_CMD_ME_CAL)
  {
    printf("  Komut 0x%02X cevabi: status %u\r\n", resp->command, status);
    return;
  }

  /* Configure cevabında sadece status anlamlı; ayarın tuttuğunu tekrar okuyarak görelim. */
  if (cal_state == CAL_WAIT_SET)
  {
    printf("  ME kalibrasyon ayari: status %u (%s) -> dogrulamak icin tekrar okunuyor\r\n",
           status, (status == 0U) ? "basarili" : "HATA");
    cal_state = CAL_SEND_GET;
    return;
  }

  uint8_t cal = bno085_cal_flags(resp);
  printf("  ME kalibrasyon (status %u, %lu ms):", status, (unsigned long)(HAL_GetTick() - cal_t0));
  print_cal_flags(cal);
  if (cal_state != CAL_WAIT_GET)
  {
    return;                                   /* beklemediğimiz bir cevap: sadece bas */
  }

  if ((cal & BNO085_CAL_GYRO) == 0U && !cal_set_tried)
  {
    /* Mevcut ayarları koru, sadece gyro'yu ekle. */
    printf("  -> gyro dinamik kalibrasyonu KAPALI, aciliyor\r\n");
    cal_flags = cal | BNO085_CAL_GYRO;
    cal_state = CAL_SEND_SET;
  }
  else
  {
    if ((cal & BNO085_CAL_GYRO) == 0U)
    {
      printf("UYARI: gyro kalibrasyonu acilamadi (Configure sonrasi hala kapali)\r\n");
    }
    cal_state = CAL_SEND_DCD_OFF;
  }
}

static void print_cal_flags(uint8_t f)
{
  printf(" accel=%s gyro=%s mag=%s planar=%s on-table=%s\r\n",
         (f & BNO085_CAL_ACCEL)    ? "ACIK" : "kapali",
         (f & BNO085_CAL_GYRO)     ? "ACIK" : "kapali",
         (f & BNO085_CAL_MAG)      ? "ACIK" : "kapali",
         (f & BNO085_CAL_PLANAR)   ? "ACIK" : "kapali",
         (f & BNO085_CAL_ON_TABLE) ? "ACIK" : "kapali");
}

/* Yazma sonucunu işler: hata varsa anlamlı mesaj basar; yazarken (full-duplex)
   sensörden paket geldiyse onu da normal paket gibi işler. true = paket gitti. */
static bool after_write(bno085_status_t st, uint16_t len, const char *what)
{
  if (st != BNO_OK)
  {
    printf("HATA: %s gonderilemedi (%d)%s\r\n", what, (int)st,
           (st == BNO_ERR_TIMEOUT) ? " -> WAKE (PF13) low iken INT 10 ms'de low olmadi" : "");
    return false;
  }
  if (len > SHTP_HEADER_LEN)
  {
    printf("   yazarken gelen paket (full-duplex):\r\n");
    handle_packet(len);
  }
  return true;
}

/* ch2'ye Product ID Request: [0xF9, 0x00]. Cevap (0xF8) ana döngüde okunur. */
static void request_product_id(void)
{
  static const uint8_t req[2] = { SH2_PRODUCT_ID_REQ, 0x00U };   /* [1] reserved */
  uint16_t len;

  printf("-> Product ID istegi (ch2: F9 00)\r\n");
  pid_t0    = HAL_GetTick();
  pid_state = PID_WAITING;

  bno085_status_t st = bno085_write_packet(SHTP_CH_CONTROL, req, sizeof(req),
                                           bno_rx, sizeof(bno_rx), &len);
  if (!after_write(st, len, "Product ID istegi"))
  {
    pid_state = PID_DONE;
  }
}

/* reports[] listesindeki sıradaki raporu REPORT_INTERVAL_US periyotla açar.
   Sensör ch2'de 0xFC ile kabul ettiği periyodu bildirir, raporlar ch3'ten akar. */
static void enable_next_report(void)
{
  uint16_t len;
  uint8_t  id = reports[reports_sent].id;

  printf("-> Set Feature: %s (0x%02X), %lu us\r\n",
         reports[reports_sent].name, id, (unsigned long)REPORT_INTERVAL_US);
  reports_sent++;
  bno085_status_t st = bno085_enable_report(id, REPORT_INTERVAL_US, bno_rx, sizeof(bno_rx), &len);
  (void)after_write(st, len, "Set Feature");

  if (reports_sent == NUM_REPORTS)
  {
    printf("   acc = accuracy durumu: 0 guvenilmez, 1 dusuk, 2 orta, 3 yuksek\r\n");
  }
}

/* count: raporun şimdiye kadarki toplam sayısı. Pencere dolunca hızı günceller. */
static void rate_update(rate_t *r, uint32_t count, uint32_t now)
{
  if (r->t0 == 0U)
  {
    r->t0 = now;                              /* ilk çağrı: pencereyi başlat */
    r->n0 = count;
  }
  else if (now - r->t0 >= RATE_WINDOW_MS)
  {
    r->hz = (count - r->n0) * 1000U / (now - r->t0);
    r->t0 = now;
    r->n0 = count;
  }
}

static void print_hz(const rate_t *r)
{
  if (r->hz > 0U)
  {
    printf("%3lu", (unsigned long)r->hz);
  }
  else
  {
    printf(" ??");                            /* ilk 1 s: henüz ölçülmedi */
  }
}

/* İki satır basar:
     1) Rotation Vector: yaw/pitch/roll + quaternion + heading hatası
     2) Accelerometer (m/s², |a|) + kalibre gyro (°/s) + manyetik alan büyüklüğü (µT)
   Her sensörün accuracy durumu (acc) ve gerçek geliş hızı (Hz) ayrı ayrı. */
static void print_orientation(void)
{
  static rate_t rv_rate, acc_rate, gyr_rate;
  uint32_t now = HAL_GetTick();
  rate_update(&rv_rate,  imu.rv.meta.count,    now);
  rate_update(&acc_rate, imu.accel.meta.count, now);
  rate_update(&gyr_rate, imu.gyro.meta.count,  now);

  /* 1) Yönelim */
  float yaw, pitch, roll;
  bno085_quat_to_euler(&imu.rv, &yaw, &pitch, &roll);

  /* RV örneğinin zamanı: saniye.mikrosaniye (nano printf %llu desteklemiyor -> iki parça) */
  uint64_t t = imu.rv.meta.t_us;
  printf("t=%5lu.%06lu ", (unsigned long)(t / 1000000U), (unsigned long)(t % 1000000U));
  print_fixed("yaw=",           yaw,         2, 7);
  print_fixed(" pitch=",        pitch,       2, 7);
  print_fixed(" roll=",         roll,        2, 7);
  print_fixed(" | q(w,x,y,z)=", imu.rv.real, 4, 7);
  print_fixed(",",              imu.rv.i,    4, 7);
  print_fixed(",",              imu.rv.j,    4, 7);
  print_fixed(",",              imu.rv.k,    4, 7);
  printf(" | acc=%u", imu.rv.status);
  print_fixed(" +-", imu.rv.accuracy_rad * RAD2DEG, 1, 5);
  printf(" deg | ");
  print_hz(&rv_rate);
  printf(" Hz\r\n");

  /* 2) Ham sensörler. |a| = ivme vektörünün boyu: sensör dururken sadece
     yerçekimini ölçer -> yönünden bağımsız ~9.81 m/s² olmalı. */
  const bno085_vec3_t *a = &imu.accel;
  const bno085_vec3_t *g = &imu.gyro;
  float a_mag = sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);

  print_fixed("    ivme=", a->x, 2, 6);
  print_fixed(",",         a->y, 2, 6);
  print_fixed(",",         a->z, 2, 6);
  print_fixed(" m/s2 |a|=", a_mag, 2, 5);
  printf(" acc=%u", a->status);
  print_fixed(" | gyro=", g->x * RAD2DEG, 2, 7);  /* rad/s -> °/s */
  print_fixed(",",        g->y * RAD2DEG, 2, 7);
  print_fixed(",",        g->z * RAD2DEG, 2, 7);
  printf(" dps acc=%u", g->status);
  const bno085_vec3_t *m = &imu.mag;
  print_fixed(" | mag |B|=", sqrtf(m->x * m->x + m->y * m->y + m->z * m->z), 1, 5);
  printf(" uT acc=%u | ", m->status);
  print_hz(&acc_rate);
  printf("/");
  print_hz(&gyr_rate);
  printf(" Hz\r\n");
}

/* Saniyelik özet: her sensör için örnekler arası süre (dt) ve seq kayıpları.
     dt = ardışık iki raporun SENSÖR zaman damgaları farkı (host'un okuma anı değil).
     Nominal: RV ve gyro 10000 us, accel 8000 us. */
static void print_timing(void)
{
  printf("    zaman:");
  print_meta(" RV",  &imu.rv.meta);
  print_meta(" | ACC", &imu.accel.meta);
  print_meta(" | GYR", &imu.gyro.meta);
  print_meta(" | MAG", &imu.mag.meta);
  printf("\r\n");

  if (bno085_stale_reads() > 0U)
  {
    printf("    UYARI: eski INT referansiyla okunan paket: %lu\r\n", (unsigned long)bno085_stale_reads());
  }

  /* Yeni bir "zaman geri gitti" olayı olduysa ham bileşenlerini bas. */
  static uint32_t tback_seen = 0;
  const bno085_tback_t *tb = bno085_tback_info();
  if (tb->count != tback_seen)
  {
    tback_seen = tb->count;
    printf("    GERI detay: rapor 0x%02X seq %u\r\n      once : t=", tb->report_id, tb->seq);
    print_sec(tb->t_prev_us);
    printf(" INT=");
    print_sec(tb->int_prev_us);
    printf(" ref=%ld delay=%u (x100 us)\r\n      sonra: t=", (long)tb->ref_prev, tb->delay_prev);
    print_sec(tb->t_new_us);
    printf(" INT=");
    print_sec(tb->int_new_us);
    printf(" ref=%ld delay=%u (x100 us) | okuma gecikmesi %lu us, referans eski: %s\r\n",
           (long)tb->ref_new, tb->delay_new, (unsigned long)tb->lat_new_us,
           tb->stale_new ? "EVET" : "hayir");
  }
}

/* Bir sensörün dt ort/min/max'ını ve toplam kaybını basar, dt penceresini sıfırlar. */
static void print_meta(const char *name, bno085_meta_t *m)
{
  if (m->dt_n == 0U)
  {
    printf("%s dt= -", name);
  }
  else
  {
    printf("%s dt=%lu (%lu..%lu) us", name, (unsigned long)(m->dt_sum / m->dt_n),
           (unsigned long)m->dt_min, (unsigned long)m->dt_max);
  }
  printf(" kayip %lu", (unsigned long)m->lost);
  if (m->t_back > 0U)
  {
    printf(" GERI %lu", (unsigned long)m->t_back);   /* zaman damgası geri gitti: olmamalı */
  }
  m->dt_n   = 0;
  m->dt_sum = 0;
}

/* newlib-nano'da float printf kapalı: v'yi ölçekli integer'a çevirip "-123.45"
   gibi basar. decimals = 1..4 ondalık hane, width = sağa yaslama genişliği. */
static void print_fixed(const char *label, float v, uint8_t decimals, int width)
{
  int      dec   = (decimals < 1U) ? 1 : (decimals > 4U) ? 4 : (int)decimals;
  uint32_t scale = 1U;
  for (int d = 0; d < dec; d++)
  {
    scale *= 10U;
  }
  float    a = (v < 0.0f) ? -v : v;
  uint32_t x = (uint32_t)(a * (float)scale + 0.5f);   /* +0.5: en yakına yuvarla */

  char s[24];                                         /* "-" + 10 hane + "." + 10 hane + NUL */
  snprintf(s, sizeof(s), "%s%lu.%0*lu", (v < 0.0f && x != 0U) ? "-" : "",
           (unsigned long)(x / scale), dec, (unsigned long)(x % scale));
  printf("%s%*s", label, width, s);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Kurtarılamayan hata: LD1 (PC7) hızlı yanıp sönsün. UART o an hazır olmayabilir
     (ör. SystemClock_Config'te LSE kristali başlamazsa daha MX_*_Init çalışmadan
     buraya düşülür), LED ise her durumda görünür. Interrupt'lar kapalı olduğu için
     HAL_Delay (SysTick) çalışmaz: gecikme boş döngüyle, o anki saat hızına göre. */
  __disable_irq();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin   = Onboard_LED_Pin;
  led.Mode  = GPIO_MODE_OUTPUT_PP;
  led.Pull  = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Onboard_LED_GPIO_Port, &led);
  while (1)
  {
    HAL_GPIO_TogglePin(Onboard_LED_GPIO_Port, Onboard_LED_Pin);
    for (volatile uint32_t i = 0; i < SystemCoreClock / 40U; i++)
    {
      /* ~1/8 s: döngü başına birkaç cycle */
    }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

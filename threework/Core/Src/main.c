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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>

typedef struct {
  uint8_t  run;       /* 0暂停 / 1运行 */
  uint8_t  dir;       /* 0正向 / 1反向 */
  uint16_t step_ms;   /* 相邻LED切换间隔 100~2000 */
  uint8_t  led_idx;   /* 当前点亮 0/1/2 (对应 LED0/1/2) */
} LightState;

LightState g_state = { 1, 0, 300, 0 };   /* 上电默认：运行、正向、300ms、亮LED0 */
volatile uint8_t g_state_dirty = 1;      /* =1 需刷OLED+发反馈，上电先刷一次 */

uint8_t  rx_byte;                        /* 单字节接收落地点 */
uint8_t  rx_buf[6];                      /* 一帧6字节 */
uint8_t  rx_cnt = 0;
volatile uint8_t g_pkt_ready = 0;        /* 攒满一帧置1，主循环解析后清0 */

uint8_t  key_dir_last = 1, key_run_last = 1;   /* 空闲电平(按下接地+上拉=1) */
uint32_t key_dir_time = 0, key_run_time = 0;
#define DEBOUNCE_MS 50

uint32_t led_last_time = 0;              /* 上次切灯时刻 */
char     tx_line[64];

#define LED_NUM 3
const GPIO_TypeDef* LED_PORT[LED_NUM] = { LED0_GPIO_Port, LED1_GPIO_Port, LED2_GPIO_Port };
const uint16_t      LED_PIN [LED_NUM] = { LED0_Pin, LED1_Pin, LED2_Pin };

/* ---- 前向声明（解决 CLion implicit declaration 报错） ---- */
static void led_apply(void);
static void led_flow(void);
static void scan_keys(void);
static void handle_packet(void);
static void uart_send_feedback(void);
static void OLED_refresh_state(void);
static void refresh_all(void);
/* 若用现成驱动库，注释掉下面两行，并 #include "oled.h" */
//void OLED_ShowString(uint8_t x, uint8_t page, const char *s);
void OLED_Clear(void);
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  led_apply();                                  // 先让初始灯亮起来
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    OLED_Init();                  // <--- 【必须加这一句】初始化屏幕
    OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    scan_keys();       // 按键改 g_state，置 dirty
    handle_packet();   // 串口包解析，改 g_state，置 dirty
    led_flow();        // 按 step_ms 推进 led_idx（run=0 跳过）
    refresh_all();     // dirty 时刷OLED+发反馈
      OLED_refresh_state();

      HAL_Delay(5);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void led_apply(void) {
    for (int i = 0; i < LED_NUM; i++)
        HAL_GPIO_WritePin((GPIO_TypeDef*)LED_PORT[i], LED_PIN[i],
                          (i == g_state.led_idx) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void led_flow(void) {
    if (g_state.run == 0) return;
    if (HAL_GetTick() - led_last_time >= g_state.step_ms) {
        led_last_time = HAL_GetTick();
        if (g_state.dir == 0) g_state.led_idx = (g_state.led_idx + 1) % LED_NUM;
        else                  g_state.led_idx = (g_state.led_idx + LED_NUM - 1) % LED_NUM;
        led_apply();
    }
}

static void scan_keys(void) {
    uint32_t now = HAL_GetTick();
    GPIO_PinState lv;

    lv = HAL_GPIO_ReadPin(KEY_DIR_GPIO_Port, KEY_DIR_Pin);     /* 切方向 */
    if (lv != key_dir_last) { key_dir_last = lv; key_dir_time = now; }
    else if (lv == GPIO_PIN_RESET && key_dir_time != 0xFFFFFFFF
             && (now - key_dir_time) >= DEBOUNCE_MS) {
        key_dir_time = 0xFFFFFFFF;
        g_state.dir ^= 1; g_state_dirty = 1;
    }

    lv = HAL_GPIO_ReadPin(KEY_RUN_GPIO_Port, KEY_RUN_Pin);     /* 切暂停/继续 */
    if (lv != key_run_last) { key_run_last = lv; key_run_time = now; }
    else if (lv == GPIO_PIN_RESET && key_run_time != 0xFFFFFFFF
             && (now - key_run_time) >= DEBOUNCE_MS) {
        key_run_time = 0xFFFFFFFF;
        g_state.run ^= 1; g_state_dirty = 1;
    }
}

static void handle_packet(void) {
    if (!g_pkt_ready) return;
    g_pkt_ready = 0;
    uint8_t sum = (uint8_t)(0xAA + rx_buf[1] + rx_buf[2] + rx_buf[3] + rx_buf[4]);
    if (rx_buf[0] == 0xAA && sum == rx_buf[5]) {
        uint16_t step = (uint16_t)(rx_buf[3] | (rx_buf[4] << 8));
        if (step >= 100 && step <= 2000) {
            g_state.run     = rx_buf[1] ? 1 : 0;
            g_state.dir     = rx_buf[2] ? 1 : 0;
            g_state.step_ms = step;
            g_state_dirty   = 1;
        }
    }
}

static void uart_send_feedback(void) {
    int n = snprintf(tx_line, sizeof(tx_line),
        "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
        g_state.run ? "RUN" : "PAUSE",
        g_state.dir ? "REV" : "FWD",
        (unsigned)g_state.step_ms,
        (unsigned)(g_state.led_idx + 1));
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_line, n, 100);  /* 阻塞发送，稳妥 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        rx_buf[rx_cnt++] = rx_byte;
        if (rx_cnt >= 6) { rx_cnt = 0; g_pkt_ready = 1; }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

static void OLED_refresh_state(void) {
    char buf[40];
    OLED_Clear();
  //  OLED_ShowString(1, 1, "STATE:");
    OLED_ShowString(1, 2, g_state.run ? "RUN" : "PAUSE");
   // OLED_ShowString(2, 1, "DIR:");
    OLED_ShowString(2, 2, g_state.dir ? "REV" : "FWD");
    //snprintf(buf, sizeof(buf), "STEP: %dms", (int)g_state.step_ms); OLED_ShowString(4, 0, buf);
    sprintf(buf, "%dms", (int)g_state.step_ms);
  //  OLED_ShowString(3, 1, "STEP: ");
    OLED_ShowString(3, 1, buf);
    //snprintf(buf, sizeof(buf), "LED:  %d", (int)(g_state.led_idx + 1)); OLED_ShowString(6, 0, buf);
    sprintf(buf, "%d", (int)(g_state.led_idx + 1));
    OLED_ShowString(4, 1, "LED:");
    OLED_ShowString(4, 20, buf);
}


static void refresh_all(void) {
    if (!g_state_dirty) return;
    g_state_dirty = 0;
    OLED_refresh_state();
    uart_send_feedback();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

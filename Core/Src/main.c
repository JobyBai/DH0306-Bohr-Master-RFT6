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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sfud.h"
#include <string.h>
#include "hmi_control.h"
#include "sensor_control.h"
#include "esp32.h"
#include "power_control.h"
#include "drv_can.h"

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
//sfud_flash *g_flash = NULL; /* flash读写全局指针 */
sfud_err sfud_result;
const sfud_flash *g_flash;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	// 初始化 SFUD 设备
	printf("\r\n====== SFUD 初始化 ======\r\n");
	/* -------- 第一步：初始化 -------- */
	sfud_result = sfud_init();
	if (sfud_result != SFUD_SUCCESS) {
		printf("[FAIL] sfud_init 失败，错误码: %d\r\n", sfud_result);
		printf("  → 检查 sfud_port.c 的 SPI 读写函数和 CS 引脚1\r\n");
		return 0;
	}
	printf("[PASS] sfud_init 成功\r\n");

	/* -------- 第二步：获取设备 -------- */
	g_flash = sfud_get_device(SFUD_W25Q128_DEVICE_INDEX);  // 换成你的设备索引
	if (g_flash == NULL) {
		printf("[FAIL] 获取 Flash 设备失败\r\n");
		printf("  → 检查 sfud_cfg.h 的 SFUD_FLASH_DEVICE_TABLE 配置\r\n");
		return 0;
	}
	printf("[PASS] 获取设备成功: %s\r\n", g_flash->name);
	printf("       容量: %lu KB\r\n", g_flash->chip.capacity / 1024);
	printf("       写粒度: %u Bytes\r\n", g_flash->chip.write_mode);

  HAL_Delay(2000);
  // 获取flash数据
  GetFlashData();

  /*启动定时器4中断*/
  HAL_TIM_Base_Start_IT(&htim4);
  Sensor_Control_Init();
  Sensor_Control_Start_Mode1();

  //初始化电机周期
  HAL_UART_Transmit(&huart4, motor_cycle_msg, 6, 100);

  // 获取配置信息
  //   GetConfigInfo();
  // CAN通讯
// DRV_CAN_InitFilterList16(0x00A << 5, 0x00A << 5, 0x00A << 5, 0x00A << 5);
  DRV_CAN_InitFilterMask16(0x00, 0x00, 0x00, 0x00);
  DRV_CAN_Start(); // 开始CAN传输
  Control_Init(); // 初始化CAN接收处理回调函数

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
	touch_hmi_task_callback();	//串口屏幕控制
	ESP32_232_communication_task_callback();// 处理ESP32从机指令
	// Sensor_Poll();// spi 接收数据
	power_communication_task_callback();// 处理电源板从机指令
	FilterByDampedNum(damping_num);
	DRV_CAN_ProcessData();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
// 用这个替换 fputc，newlib-nano 走的是 _write
int _write(int fd, char *pBuffer, int size) {
	HAL_UART_Transmit(&huart1, (uint8_t*) pBuffer, size, HAL_MAX_DELAY);
	return size;
}

void sfud_test(void) {
	sfud_err result;
	const sfud_flash *flash;

	printf("\r\n====== SFUD 移植测试开始 ======\r\n");

	/* -------- 第一步：初始化 -------- */
	result = sfud_init();
	if (result != SFUD_SUCCESS) {
		printf("[FAIL] sfud_init 失败，错误码: %d\r\n", result);
		printf("  → 检查 sfud_port.c 的 SPI 读写函数和 CS 引脚\r\n");
		return;
	}
	printf("[PASS] sfud_init 成功\r\n");

	/* -------- 第二步：获取设备 -------- */
	flash = sfud_get_device(SFUD_W25Q128_DEVICE_INDEX);  // 换成你的设备索引
	if (flash == NULL) {
		printf("[FAIL] 获取 Flash 设备失败\r\n");
		printf("  → 检查 sfud_cfg.h 的 SFUD_FLASH_DEVICE_TABLE 配置\r\n");
		return;
	}
	printf("[PASS] 获取设备成功: %s\r\n", flash->name);
	printf("       容量: %lu KB\r\n", flash->chip.capacity / 1024);
	printf("       写粒度: %u Bytes\r\n", flash->chip.write_mode);

	/* -------- 第三步：擦除测试 -------- */
	uint32_t test_addr = 0x000000;   // 测试用的起始地址（第0扇区）
	uint32_t test_size = 4096;       // 一个扇区大小

	printf("\r\n[TEST] 擦除地址 0x%06lX，大小 %lu Bytes...\r\n", test_addr, test_size);
	result = sfud_erase(flash, test_addr, test_size);
	if (result != SFUD_SUCCESS) {
		printf("[FAIL] 擦除失败，错误码: %d\r\n", result);
		return;
	}
	printf("[PASS] 擦除成功\r\n");

	/* -------- 第四步：擦除校验（读回应为 0xFF）-------- */
	uint8_t verify_buf[64];
	result = sfud_read(flash, test_addr, sizeof(verify_buf), verify_buf);
	if (result != SFUD_SUCCESS) {
		printf("[FAIL] 擦除后读取失败，错误码: %d\r\n", result);
		return;
	}
	uint8_t erase_ok = 1;
	for (int i = 0; i < sizeof(verify_buf); i++) {
		if (verify_buf[i] != 0xFF) {
			erase_ok = 0;
			break;
		}
	}
	if (!erase_ok) {
		printf("[FAIL] 擦除校验失败，数据不全为 0xFF\r\n");
		printf("  → 检查 SPI 时序 / Flash 型号是否匹配\r\n");
		return;
	}
	printf("[PASS] 擦除校验通过（全 0xFF）\r\n");

	/* -------- 第五步：写入测试 -------- */
	uint8_t write_buf[64];
	for (int i = 0; i < sizeof(write_buf); i++)
		write_buf[i] = i;  // 写入 0x00~0x3F

	printf("\r\n[TEST] 写入 %d Bytes 到地址 0x%06lX...\r\n", sizeof(write_buf),
			test_addr);
	// result = sfud_write(flash, test_addr, sizeof(write_buf), write_buf);
	sfud_erase_write(flash, test_addr, sizeof(write_buf), write_buf);
	if (result != SFUD_SUCCESS) {
		printf("[FAIL] 写入失败，错误码: %d\r\n", result);
		return;
	}
	printf("[PASS] 写入成功\r\n");

	/* -------- 第六步：读回校验 -------- */
	uint8_t read_buf[64];
	memset(read_buf, 0, sizeof(read_buf));

	result = sfud_read(flash, test_addr, sizeof(read_buf), read_buf);
	if (result != SFUD_SUCCESS) {
		printf("[FAIL] 读取失败，错误码: %d\r\n", result);
		return;
	}

	if (memcmp(write_buf, read_buf, sizeof(write_buf)) != 0) {
		printf("[FAIL] 读写数据不一致！\r\n");
		printf("  写入: ");
		for (int i = 0; i < 16; i++)
			printf("%02X ", write_buf[i]);
		printf("\r\n  读回: ");
		for (int i = 0; i < 16; i++)
			printf("%02X ", read_buf[i]);
		printf("\r\n  → 检查 SPI CPOL/CPHA 极性，或 DMA/Cache 对齐问题\r\n");
		return;
	}
	printf("[PASS] 读写校验通过\r\n");

	/* -------- 测试结束 -------- */
	printf("\r\n====== 所有测试通过，移植成功！======\r\n");
}

void test_flash_jedec_id(void) {
	uint8_t cmd = 0x9F;          // Read JEDEC ID 命令
	uint8_t id[3] = { 0 };

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	HAL_SPI_Receive(&hspi1, id, 3, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

	printf("JEDEC ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);
	// W25Q128BV 正确结果应为: EF 40 18
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
	while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

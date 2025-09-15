/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
/* ====================== KHAI BÁO & CẤU HÌNH ====================== */
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "SEGGER_SYSVIEW.h"
#include <stdio.h>
#include "fonts.h"
#include "ssd1306.h"
#include "nrf24l01.h"
//===========================================

#define SIZE_RX_BUF 8
uint8_t rx_data[SIZE_RX_BUF];
uint8_t rxAddr[] = { 0xEA, 0xDD, 0xCC, 0xBB, 0xAA };
nrf24 nrfRx;


//=============================================
/* Biến toàn cục lưu giá trị góc */
float pitch_val, roll_val, yaw_val; // Tích phân từ gyro Z

/* Biến lưu dữ liệu gia tốc và gyro */
float AX, AY, AZ, GX, GY, GZ;
int16_t ax = 0, ay = 0, az = 0;   // raw accel
int16_t gx = 0, gy = 0, gz = 0;   // raw gyro

float pitch = 0.0f, roll = 0.0f;  // góc tính toán ban đầu
uint32_t lastTime = 0;            // thời điểm đọc lần trước

uint8_t check;    // dùng để đọc WHO_AM_I (địa chỉ ID MPU6050)
uint8_t mData;    // buffer tạm để ghi vào thanh ghi MPU6050

/* ====================== KALMAN FILTER ====================== */
/* Cấu trúc dữ liệu Kalman */
typedef struct {
	float angle;   // góc ước lượng
	float bias;    // lỗi bias gyro
	float rate;    // tốc độ góc sau khi trừ bias
	float P[2][2]; // ma trận covariance
} Kalman_t;

/* Khởi tạo filter cho pitch và roll */
Kalman_t kalmanPitch = { 0 }, kalmanRoll = { 0 };

/* Tham số Kalman */
float Q_angle = 0.001f;   // noise quá trình (cho góc)
float Q_bias = 0.003f;   // noise quá trình (cho bias)
float R_measure = 0.03f;    // noise đo (từ cảm biến accel)

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
//================================================================================
float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt) {
	/* ======= Bước 1: Dự đoán (Predict) ======= */
	Kalman->rate = newRate - Kalman->bias;
	Kalman->angle += dt * Kalman->rate;

	// Cập nhật ma trận sai số
	Kalman->P[0][0] += dt
			* (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0]
					+ Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Q_bias * dt;

	/* ======= Bước 2: Cập nhật (Update) ======= */
	float S = Kalman->P[0][0] + R_measure; // Độ bất định đo lường
	float K[2];                            // Kalman gain
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	float y = newAngle - Kalman->angle; // Sai số đo
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	// Cập nhật ma trận covariance
	float P00_temp = Kalman->P[0][0];
	float P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void TaskMPU6050_Read(void *argument);
void TaskOLED_Display(void *argument);
void TaskNRF_RX(void *argument);
//==========================================================
void MPU6050Init(void);
void MPU6050ReadA(void);
void MPU6050ReadG(void);
void vSystemViewStartTask(void *argument);
void vConfigureTimerForRunTimeStats(void);
void SystemClock_Config(void);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	//=============================================================================
	nrfRx.hSPIx   = &hspi1;
	nrfRx.CE_port =CE_GPIO_Port;
	nrfRx.CE_pin  = CE_Pin;
	nrfRx.CSN_port= CSN_GPIO_Port;
	nrfRx.CSN_pin = CSN_Pin;

	nrfRx.hSPIx = &hspi1;
	NRF24_Init(&nrfRx);
	NRF24_Set_DataRate(&nrfRx, _250KBS);
	NRF24_Set_PALevel(&nrfRx, HIGH);
	NRF24_Set_RxPipe(&nrfRx, rxAddr, 0, SIZE_RX_BUF);
	NRF24_Set_Mode(&nrfRx, RX_MODE);
	//=========================================================================
	vConfigureTimerForRunTimeStats();
	SEGGER_SYSVIEW_Conf(); /* Thêm vào nếu sử dụng SEGGER SystemView!*/
	MPU6050Init();
	xTaskCreate(TaskMPU6050_Read, "MPU6050", 256, NULL, 2, NULL);
	xTaskCreate(TaskOLED_Display, "OLED", 256, NULL, 1, NULL);
	xTaskCreate(TaskNRF_RX, "NRF_RX", 256, NULL, 3, NULL); // priority cao hơn OLED
	// Bắt đầu RTOS
	/* Tạo task khởi động SystemView */
	xTaskCreate(vSystemViewStartTask, "SysViewStart", 256, NULL,
	configMAX_PRIORITIES - 1, NULL);

	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//====================================SystemView=================================
void vSystemViewStartTask(void *argument) {
	// THÊM DÒNG NÀY: Chờ 1 giây để J-Link và SystemView trên PC sẵn sàng
	vTaskDelay(pdMS_TO_TICKS(100));

	/* Bật trace, DWT, ITM an toàn khi đang debug */
	if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

		ITM->LAR = 0xC5ACCE55;
		ITM->TCR = 0;
		ITM->TCR = ITM_TCR_ITMENA_Msk | (1 << ITM_TCR_TSENA_Pos);
		ITM->TER = 1;

		SEGGER_SYSVIEW_Start();
	}
	/* Task này chỉ chạy 1 lần và tự xóa */
	vTaskDelete(NULL);
}
void vConfigureTimerForRunTimeStats(void) {
	// Bật clock cho TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// Cấu hình TIM2 chạy ở 1 MHz
	TIM2->ARR = 0xFFFFFFFF; // Đếm tối đa
	TIM2->PSC = (SystemCoreClock / 1000000) - 1; // Prescaler để đạt 1 MHz
	TIM2->CR1 |= TIM_CR1_CEN; // Bật Timer
}

unsigned long ulGetRunTimeCounterValue(void) {
	return TIM2->CNT; // Trả về giá trị đếm
}
//=======================================MPU6050Init=======================================
void MPU6050Init(void) {
	// Đọc thanh ghi WHO_AM_I (0x75), kết quả phải là 0x68
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x75, I2C_MEMADD_SIZE_8BIT, &check, 1, 1000);

	if (check == 0x68) {
		// Thoát chế độ sleep (ghi 0 vào thanh ghi PWR_MGMT_1)
		mData = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x6B, I2C_MEMADD_SIZE_8BIT, &mData, 1,
				1000);

		// Cấu hình sample rate: 1kHz / (1+7) = 125Hz
		mData = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x19, I2C_MEMADD_SIZE_8BIT, &mData, 1,
				1000);

		// Cấu hình thang đo gyro ±250 °/s
		mData = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x1B, I2C_MEMADD_SIZE_8BIT, &mData, 1,
				1000);

		// Cấu hình thang đo accel ±2g
		mData = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x1C, I2C_MEMADD_SIZE_8BIT, &mData, 1,
				1000);
	}
}
//====================================ReadG=======================================
void MPU6050ReadG(void) {
	uint8_t dataG[6];
	// Đọc 6 byte từ thanh ghi GYRO_XOUT_H (0x43)
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x43, I2C_MEMADD_SIZE_8BIT, dataG, 6, 1000);

	// Gộp byte high và low thành số 16-bit
	gx = (int16_t) (dataG[0] << 8 | dataG[1]);
	gy = (int16_t) (dataG[2] << 8 | dataG[3]);
	gz = (int16_t) (dataG[4] << 8 | dataG[5]);

	// Chuyển đổi sang °/s (scale ±250°/s → 131 LSB/°/s)
	GX = (float) gx / 131.0;
	GY = (float) gy / 131.0;
	GZ = (float) gz / 131.0;
}
//================================ReadA================================
void MPU6050ReadA(void) {
	uint8_t dataA[6];
	// Đọc 6 byte từ thanh ghi ACCEL_XOUT_H (0x3B)
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x3B, I2C_MEMADD_SIZE_8BIT, dataA, 6, 1000);

	ax = (int16_t) (dataA[0] << 8 | dataA[1]);
	ay = (int16_t) (dataA[2] << 8 | dataA[3]);
	az = (int16_t) (dataA[4] << 8 | dataA[5]);

	// Chuyển đổi sang đơn vị g (scale ±2g → 16384 LSB/g)
	AX = (float) ax / 16384.0;
	AY = (float) ay / 16384.0;
	AZ = (float) az / 16384.0;
}
//==================================TaskMPU6050==========================
void TaskMPU6050_Read(void *argument) {
	lastTime = HAL_GetTick();

	for (;;) {
		// Đọc cảm biến
		MPU6050ReadA();
		MPU6050ReadG();

		// Tính khoảng thời gian delta t
		uint32_t now = HAL_GetTick();
		float dt = (now - lastTime) / 1000.0f;
		lastTime = now;

		// Góc tính từ accelerometer (độ)
		float pitchAcc = atan2f(AX, sqrtf(AY * AY + AZ * AZ)) * 57.2958f;
		float rollAcc = atan2f(AY, sqrtf(AX * AX + AZ * AZ)) * 57.2958f;

		// Lọc Kalman (kết hợp gyro + accel)
		pitch_val = Kalman_getAngle(&kalmanPitch, pitchAcc, GX, dt);
		roll_val = Kalman_getAngle(&kalmanRoll, rollAcc, GY, dt);
		// Tích phân từ gyro Z
		yaw_val += GZ * dt;   // GZ: deg/s, dt: thời gian (s)

		// Debug (in ra UART nếu cần)
		// printf("Pitch: %.2f, Roll: %.2f\r\n", pitch_val, roll_val);

		// Delay 10ms → tần số 100Hz
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
//======================================TaskOLED_Display=======================
void TaskOLED_Display(void *argument) {
	SSD1306_Init();
	char buf[32];

	for (;;) {
		// Xóa màn hình
		SSD1306_Clear();

		// In Pitch
		sprintf(buf, "Pitch: %.2f", pitch_val);
		SSD1306_GotoXY(0, 0);
		SSD1306_Puts(buf, &Font_7x10, 1);

		// In Roll
		sprintf(buf, "Roll : %.2f", roll_val);
		SSD1306_GotoXY(0, 12);
		SSD1306_Puts(buf, &Font_7x10, 1);

		// In Yaw
		sprintf(buf, "Yaw  : %.2f", yaw_val);
		SSD1306_GotoXY(0, 24);
		SSD1306_Puts(buf, &Font_7x10, 1);

		// Cập nhật màn hình
		SSD1306_UpdateScreen();

		// Delay 200ms
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}
//===================================TaskNRF_RX=======================================
void TaskNRF_RX(void *argument)
{
    for(;;)
    {
        // Nếu có dữ liệu từ TX gửi sang
        if (NRF24_Available(&nrfRx, 0)) {
            NRF24_Receive(&nrfRx, rx_data, SIZE_RX_BUF);

            // Chỉ đảo LED PC13
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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

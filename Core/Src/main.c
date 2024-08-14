/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "lfs_util.h"
#include "lfs.h"
#include "nor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define CDC_USB_DEBUG
#define UART_DEBUG
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define sector_size 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void myprintf(const char *fmt, ...);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */
//uint8_t tx[]={'S','A','N','G','A','M'};

uint16_t address = 0x00;
uint8_t status_reg=0;
uint8_t READ_FLAG=0;


uint8_t DEBUG_DATA_RX_FLAG = 0;
// variables used by the filesystem
typedef struct{
	uint32_t secCount;
	uint32_t bootCount;
}app_count_t;
	lfs_file_t File;
		char Text[20];
		app_count_t Counter = {0};
		lfs_t Lfs;
		nor_t Nor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_DMA_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t DmaEnd = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	DmaEnd = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	DmaEnd = 1;
}

void nor_delay_us(uint32_t us){
//	if (us >= __HAL_TIM_GET_AUTORELOAD(&htim2)){
//		us = __HAL_TIM_GET_AUTORELOAD(&htim2) - 1;
//	}
//	__HAL_TIM_SET_COUNTER(&htim2, 0);
//	HAL_TIM_Base_Start(&htim2);
//	while (__HAL_TIM_GET_COUNTER(&htim2) < us);
//	HAL_TIM_Base_Stop(&htim2);
	HAL_Delay(1000);
}

void nor_cs_assert(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
}

void nor_cs_deassert(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
}

void nor_spi_tx(uint8_t *pData, uint32_t Size){
//	HAL_SPI_Transmit(&hspi3, pData, Size, 100);
	DmaEnd = 0;
	HAL_SPI_Transmit(&hspi3, pData, Size, 1000);
//	while (DmaEnd == 0);
}

void nor_spi_rx(uint8_t *pData, uint32_t Size){
//	HAL_SPI_Receive(&hspi3, pData, Size, 100);
	DmaEnd = 0;
	HAL_SPI_Receive(&hspi3, pData, Size, 1000);
//	DmaEnd =0;
//	while (DmaEnd == 0);
}

void __init_nor(){
	Nor.config.CsAssert = nor_cs_assert;
	Nor.config.CsDeassert = nor_cs_deassert;
	Nor.config.DelayUs = nor_delay_us;
	Nor.config.SpiRxFxn = nor_spi_rx;
	Nor.config.SpiTxFxn = nor_spi_tx;

	if (NOR_Init(&Nor) != NOR_OK){ //NOR_Init
		Error_Handler();
	}
}

/** Start LittleFs **/

int _fs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size){

	if (NOR_ReadSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_write(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size){

	if (NOR_WriteSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_erase(const struct lfs_config *c, lfs_block_t block){
	if (NOR_EraseSector(&Nor, block) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_sync(const struct lfs_config *c){
	return 0;
}

void __init_littefs(){
	// because of static qualifier, this variable
	// will have a dedicated address
	static struct lfs_config LfsConfig = {0};
	int Error;

	LfsConfig.read_size = 64;
	LfsConfig.prog_size = 64;
	LfsConfig.block_size = Nor.info.u16SectorSize;
	LfsConfig.block_count = Nor.info.u32SectorCount;
	LfsConfig.cache_size = Nor.info.u16PageSize;
	LfsConfig.lookahead_size = 8;//Nor.info.u32SectorCount/8;
	LfsConfig.block_cycles = 15000;

	LfsConfig.read = _fs_read;
	LfsConfig.prog = _fs_write;
	LfsConfig.erase = _fs_erase;
	LfsConfig.sync = _fs_sync;

	Error = lfs_mount(&Lfs, &LfsConfig);
	if (Error != LFS_ERR_OK){
		lfs_format(&Lfs, &LfsConfig);
		Error = lfs_mount(&Lfs, &LfsConfig);
		if (Error != LFS_ERR_OK){
			Error_Handler();
		}
	}
}

void __init_storage(){
	__init_nor();
	__init_littefs();
}
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
  MX_SPI3_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
//simple
//  //First erase flash memory
//  	Sector_Erase_4B(&hspi3, address, sector_size);
//	//id read
//	Read_ID(&hspi3, &dev_id);
//
//	HAL_Delay(100);

	//status_reg = Status_Reg(&hspi3);
	//  int add = 0;
	//  for(int i=0;i<300;i++){
	//	  Sector_Erase_4B(&hspi3, add, 64);
	//	  add+=65536;
	//  }

	  // myprintf("Starting LittleFS application........\n");
    HAL_Delay(100);

  HAL_UART_Transmit(&huart7,"EPDM is starting *********\n", sizeof("EPDM is starting *********\n"),1000);


  __init_storage();
   lfs_file_open(&Lfs, &File, "count.bin", LFS_O_RDWR | LFS_O_CREAT);
   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
   lfs_file_close(&Lfs, &File);

   Counter.bootCount += 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { sprintf(Text, "Bt %lu |Ct %lu\n", Counter.bootCount, Counter.secCount);
	  HAL_UART_Transmit(&huart7,Text, sizeof(Text),1000);

	  HAL_UART_Transmit(&huart7,"*******\n", sizeof("*******\n"),1000);

		  lfs_file_open(&Lfs, &File, "count.bin", LFS_O_RDWR | LFS_O_CREAT);
		  lfs_file_write(&Lfs, &File, &Counter, sizeof(app_count_t));
		  lfs_file_close(&Lfs, &File);

//		  while ((HAL_GetTick() - HalTickAux) < 1000);
		  HAL_Delay(1000);

		  Counter.secCount += 1;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void myprintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buffer[100];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    HAL_UART_Transmit(&huart7, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
    va_end(args);
}

int bufferSize(char *buffer) {
    int i = 0;
    while (*buffer++ != '\0')
        i++;
    return i;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

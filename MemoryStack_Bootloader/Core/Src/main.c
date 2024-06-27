/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "crc.h"
#include "spi.h"
#include "gpio.h"


#include "bootloader.h"

#include "FreeRTOS.h"
#include "task.h"


#include "Det.h"
#include "NVM.h"
#include "Fls.h"
#include "Memif.h"
#include "Fee.h"
#include "Ea.h"
#include "Eep.h"

#define WRITE_SIZE    1024
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	TaskHandle_t RedLedHandle                 = NULL;
	TaskHandle_t GreenLedHandle               = NULL;
	//TaskHandle_t NvM_MainFunctionWriteHandle  = NULL;
	TaskHandle_t BL_MemoryWriteHandle  = NULL;


	extern Fls_ConfigType Fls_Config;
	extern Eep_ConfigType Eep_Config ;
	uint8_t Wdata[WRITE_SIZE]={1};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	void Excute_bootloader_erase (void * Parameter);
	void GreenLed (void * Parameter);
	void Write_buffer(uint8_t val);

	void Excute_NvM_MainFunction (void *Parameter);
	void Excute_BL_MemoryWrite (void *Parameter);

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
uint8_t command[] = {0x0F, 0x16, 0x00,0x80, 0x00, 0x08,3,0x64,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5, 20,20,20,20};
uint8_t backet3[] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
 uint32 Dummy[24] = {1, 1,1,1,1, 1, 1,1,1,1, 1, 1,1,1,1, 1, 1,1,1,1 ,0x6BBF3ED3};
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Fls_Init(&Fls_Config);
//  Eep_Init(&Eep_Config);
  Ea_Init(NULL) ;
  Fee_Init(NULL);
  NvM_Init(NULL);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  xTaskCreate(Excute_bootloader_erase, "bootloader erase", 128, NULL, 1, &RedLedHandle);
  //xTaskCreate(GreenLed, "Green LED Task", 128, NULL, 1, &GreenLedHandle);

//  xTaskCreate(Excute_NvM_MainFunction, "NvM Main Function", 128, NULL, 1, &NvM_MainFunctionWriteHandle);
  xTaskCreate(Excute_BL_MemoryWrite, "BL Memory Write", 128, NULL, 1, &BL_MemoryWriteHandle);

  //NvM_WriteBlock(3,Dummy);

  vTaskStartScheduler();
  //Bootloader_Memory_Write(command);


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Excute_bootloader_erase (void * Parameter){


}
void GreenLed (void * Parameter){

	for(;;){

	}

}

//void Excute_NvM_MainFunction (void *Parameter){
//
//	while(1){
//		NvM_MainFunction();
//	}
//
//}

void Excute_BL_MemoryWrite (void *Parameter){

	while(1){
		Bootloader_Memory_Write(command);
	}

}

void Write_buffer(uint8 val)
{
    uint16 count =0 ;
    for(count = 0 ;count <WRITE_SIZE ;count++)
    {
        Wdata[count] = val ;
    }
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

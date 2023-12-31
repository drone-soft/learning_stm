/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId Task_K0Handle;
osThreadId Task_K1Handle;
osSemaphoreId ButtonTasksSemaphoreHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartTask_K0(void const * argument);
void StartTask_K1(void const * argument);

/* USER CODE BEGIN PFP */

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
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;//enabling A port clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;//enabling E port clock
  GPIOA->MODER |= GPIO_MODER_MODE6_0;
  GPIOA->MODER |= GPIO_MODER_MODE7_0;
  GPIOE->PUPDR |= GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD3_0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  //Sets LEDs to be turned of at the start
  GPIOA->BSRR |= GPIO_BSRR_BS7;
  GPIOA->BSRR |= GPIO_BSRR_BS6;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ButtonTasksSemaphore */
  osSemaphoreDef(ButtonTasksSemaphore);
  ButtonTasksSemaphoreHandle = osSemaphoreCreate(osSemaphore(ButtonTasksSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_K0 */
  osThreadDef(Task_K0, StartTask_K0, osPriorityNormal, 0, 128);
  Task_K0Handle = osThreadCreate(osThread(Task_K0), NULL);

  /* definition and creation of Task_K1 */
  osThreadDef(Task_K1, StartTask_K1, osPriorityNormal, 0, 128);
  Task_K1Handle = osThreadCreate(osThread(Task_K1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartTask_K0(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t tickstart;
	uint32_t wait = 3000;
  /* Infinite loop */
	 for(;;)
	  {
			  if ( (~(GPIOE->IDR) & GPIO_IDR_ID4) &&
					  !osSemaphoreWait(ButtonTasksSemaphoreHandle , osWaitForever))
			  {
				  //turning on the led
				  GPIOA->BSRR |= GPIO_BSRR_BR6;
				  osSemaphoreRelease(ButtonTasksSemaphoreHandle);
				  //turning on the led

				  //waiting
//				  tickstart = HAL_GetTick();
//				  while((HAL_GetTick() - tickstart) < wait) {}
				  osDelay(wait);
				  //waiting

				  //turning off the led
				  while(1){
					  if (!osSemaphoreWait(ButtonTasksSemaphoreHandle , osWaitForever)){
						  GPIOA->BSRR |= GPIO_BSRR_BS6;
						  osSemaphoreRelease(ButtonTasksSemaphoreHandle);
						  break;
					  }
				  }
				  //turning off the led
			  }

		  osDelay(10);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task_K1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask_K1(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	uint32_t wait = 3000;

  for(;;)
  {
	  //Button K1
		  if ( (~(GPIOE->IDR) & GPIO_IDR_ID3) &&
				  !osSemaphoreWait(ButtonTasksSemaphoreHandle , osWaitForever))
		  {
			  GPIOA->BSRR |= GPIO_BSRR_BR7;
			  osSemaphoreRelease(ButtonTasksSemaphoreHandle);

			  osDelay(wait);

			  while(1) {
				  if (!osSemaphoreWait(ButtonTasksSemaphoreHandle , osWaitForever)){
					  GPIOA->BSRR |= GPIO_BSRR_BS7;
					  osSemaphoreRelease(ButtonTasksSemaphoreHandle);
					  break;
				  }
			  }

		  }
  }
  /* USER CODE END StartTask02 */
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

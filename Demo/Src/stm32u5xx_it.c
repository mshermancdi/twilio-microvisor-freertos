/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32u5xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32u5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_threadx.h"
//#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/*extern TX_SEMAPHORE txDataReadyBinarySem;
extern TX_SEMAPHORE txXACReadyForVerifySem;
extern TX_SEMAPHORE txYACReadyForVerifySem;
extern TX_SEMAPHORE txXDCReadyForVerifySem;
extern TX_SEMAPHORE txYDCReadyForVerifySem;*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
//extern DMA_HandleTypeDef handle_GPDMA1_Channel12;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
//extern TIM_HandleTypeDef htim3;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel6;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel5;
//extern DMA_HandleTypeDef handle_GPDMA1_Channel7;
extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;
//extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}


/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32U5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI Line13 interrupt.
  */
void EXTI13_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI13_IRQn 0 */

  /* USER CODE END EXTI13_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(STMOD_UART_SEL_Pin);
  /* USER CODE BEGIN EXTI13_IRQn 1 */

  /* USER CODE END EXTI13_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */

  /* USER CODE END GPDMA1_Channel0_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */
//	tx_semaphore_put(&txDataReadyBinarySem);

  /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 1 global interrupt.
  */
void GPDMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 0 */

  /* USER CODE END GPDMA1_Channel1_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel1);
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 1 */
//	tx_semaphore_put(&txXACReadyForVerifySem);
  /* USER CODE END GPDMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 2 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 0 */

  /* USER CODE END GPDMA1_Channel2_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 1 */
//  tx_semaphore_put(&txYACReadyForVerifySem);
  /* USER CODE END GPDMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 3 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 0 */

  /* USER CODE END GPDMA1_Channel3_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel3);
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 1 */
//  tx_semaphore_put(&txXDCReadyForVerifySem);
  /* USER CODE END GPDMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */
//  tx_semaphore_put(&txYDCReadyForVerifySem);
  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 5 global interrupt.
  */
void GPDMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 0 */

  /* USER CODE END GPDMA1_Channel5_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel5);
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 1 */

  /* USER CODE END GPDMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 6 global interrupt.
  */
void GPDMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel6_IRQn 0 */

  /* USER CODE END GPDMA1_Channel6_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel6);
  /* USER CODE BEGIN GPDMA1_Channel6_IRQn 1 */

  /* USER CODE END GPDMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 7 global interrupt.
  */
void GPDMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel7_IRQn 0 */

  /* USER CODE END GPDMA1_Channel7_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel7);
  /* USER CODE BEGIN GPDMA1_Channel7_IRQn 1 */

  /* USER CODE END GPDMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
//	GPS_TimeUpdate();
  /* USER CODE END TIM3_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
//  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 12 global interrupt.
  */
void GPDMA1_Channel12_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel12_IRQn 0 */

  /* USER CODE END GPDMA1_Channel12_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel12);
  /* USER CODE BEGIN GPDMA1_Channel12_IRQn 1 */

  /* USER CODE END GPDMA1_Channel12_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "app_azure_rtos_config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "console.h"
#include "debug_menu_strings_en.h"
#include "gpdma.h"

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

// Thread declarations
//
TX_THREAD txStartupTask;
TX_THREAD txServicePortTask;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//
//	Task function prototypes
//
void StartupTask_Entry(ULONG thread_input);
void ServicePortTask_Entry(ULONG thread_input);

//	Global byte pool point declaration
TX_BYTE_POOL *pGlobal_byte_pool = TX_NULL;

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
//  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

   /* USER CODE BEGIN App_ThreadX_MEM_POOL */

	pGlobal_byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

#if (USE_MEMORY_POOL_ALLOCATION == 1)
  CHAR *pMemPool;

	/* Allocate the stack for StartupTask.  */
  if (tx_byte_allocate(pGlobal_byte_pool, (VOID **) &pMemPool,
                       STARTUP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create StartupTask.  */
  if (tx_thread_create(	&txStartupTask,
												"Startup Thread",
												StartupTask_Entry,
												0,
												pMemPool,
												STARTUP_STACK_SIZE,
												THREAD_STARTUP_PRIO,
												THREAD_STARTUP_PREEMPTION_THRESHOLD,
												TX_NO_TIME_SLICE,
												TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

  /* Allocate the stack for ServicePortTask.  */
  if (tx_byte_allocate(pGlobal_byte_pool, (VOID **) &pMemPool,
						 SERVICE_PORT_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {

	ret = TX_POOL_ERROR;

  }

  /* Create ServicePortTask.  */
  if (tx_thread_create(	&txServicePortTask,
												"Service Port Thread",
												ServicePortTask_Entry,
												0,
												pMemPool,
												SERVICE_PORT_STACK_SIZE,
												THREAD_SERVICE_PORT_PRIO,
												THREAD_SERVICE_PORT_PREEMPTION_THRESHOLD,
												TX_NO_TIME_SLICE,
												TX_AUTO_START) != TX_SUCCESS)

  {
	ret = TX_THREAD_ERROR;
  }

#endif
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
cmd_t cmdlist[] =
{
  {"help",                                         "print this message",          help_cmd},
  {"quit",                                                       "quit",          quit_cmd},
  {"tile",                                                 "swarm tile",          tile_cmd},
//  {"led",                                                    "led test",           led_cmd},
//  {"gps",                                                    "gps info",           gps_cmd},
  {  NULL,                                                         NULL,              NULL}
};

/* USER CODE BEGIN Header_StartStartupTask */
/**
  * @brief  Function implementing the startupTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStartupTask */
void StartupTask_Entry(ULONG thread_input)
{

  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	/* Sleep for 500 ms */
	tx_thread_sleep(500);
  }
}

/* USER CODE BEGIN Header_StartServicePortTask */
/**
  * @brief  Function implementing the servicePortTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartServicePortTask */
void ServicePortTask_Entry(ULONG thread_input)
{
  UNUSED(thread_input);

  /* Infinite loop */
  while (1)
  {
#if INSTRUMENTATION
		bSPTask = true;
#endif

		quit_msg = 0;
		printf("\n\n\n\n\r#### Welcome to Qube #####\n");
		printf("\n>>> Please choose one of the following commands:\n\n");
		help_cmd(0, NULL);

		/* Application body. */
		console(USER_PROMPT, cmdlist);

#if INSTRUMENTATION
		bSPTask = false;
#endif
	/* Sleep for 500 ms */
	tx_thread_sleep(500);
  }
}


/* USER CODE END 1 */

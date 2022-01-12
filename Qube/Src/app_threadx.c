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
#include "swarm.h"
#include "gps.h"

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

extern DMA_HandleTypeDef handle_GPDMA1_Channel5;
extern DMA_HandleTypeDef handle_GPDMA1_Channel6;
extern DMA_HandleTypeDef handle_GPDMA1_Channel7;

extern uint8_t MESSAGES[NUM_E_MESSAGES][MAX_DEBUG_MENU_STRLEN];

extern __IO int uart_write_idx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define swarm_uart			huart2
#define SWARM_UART			USART2
#define swarm_dma_rx		handle_GPDMA1_Channel5
#define swarm_dma_tx		handle_GPDMA1_Channel6
#define gps_uart			huart3
#define GPS_UART			USART3
#define gps_dma_rx		handle_GPDMA1_Channel7

#define SM_BUF_SZ 100
#define LG_BUF_SZ 512

uint8_t uart2Buf[SM_BUF_SZ] = {'\0'};
uint8_t swarmBuf[LG_BUF_SZ] = {'\0'};
uint16_t swarmBufCurrPos = 0;
uint16_t swarmBufLastPos = 0;
S_SWARM_DEVICE swarm;

uint8_t uart3Buf[SM_BUF_SZ] = {'\0'};
uint8_t gpsBuf[LG_BUF_SZ] = {'\0'};
uint16_t gpsBufCurrPos = 0;
uint16_t gpsBufLastPos = 0;
S_GPS_DEVICE gps;

// Thread declarations
//
TX_THREAD txStartupTask;
TX_THREAD txServicePortTask;
TX_THREAD txSwarmTask;
TX_THREAD txGPSTask;

//	Queue declarations
TX_QUEUE  SwarmMsgQueue;

//	Mutex declarations
TX_MUTEX SwarmMutex;
TX_MUTEX GPSMutex;

// Timer declarations
//TX_TIMER SatMsgRxTimer;

//	Event flags declarations
TX_EVENT_FLAGS_GROUP EventFlagsGroup;
TX_EVENT_FLAGS_GROUP SwarmDataReadyFlagsGroup;

//	Semaphore declarations
TX_SEMAPHORE txDataReadyBinarySem;

#define SAT_MSG_RX_TIMEOUT	90*TX_TIMER_TICKS_PER_SECOND	// 90 seconds

volatile uint32_t SWARM_TX_Completed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//
//	Task function prototypes
//
void StartupTask_Entry(ULONG thread_input);
void ServicePortTask_Entry(ULONG thread_input);
void SwarmTask_Entry(ULONG thread_input);

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

  /* Allocate the stack for Swarm Task.  */
  if (tx_byte_allocate(pGlobal_byte_pool, (VOID **) &pMemPool,
                       SWARM_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create SwarmTask.  */
  if (tx_thread_create(&txSwarmTask, "Swarm Task", SwarmTask_Entry, 0,
                       pMemPool, SWARM_STACK_SIZE,
                       SWARM_TASK_PRIO, SWARM_TASK_PREEMPTION_THRESHOLD,
                       TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

	/* Allocate the SwarmMsgQueue.  */
  if (tx_byte_allocate(pGlobal_byte_pool, (VOID **) &pMemPool,
                       SWARM_QUEUE_SIZE*sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create the SwarmMsgQueue shared by MsgSenderThreadOne and MsgReceiverThread */
  if (tx_queue_create(&SwarmMsgQueue, "Swarm Message Queue",TX_1_ULONG,
                      pMemPool, SWARM_QUEUE_SIZE*sizeof(ULONG)) != TX_SUCCESS)
  {
    ret = TX_QUEUE_ERROR;
  }

  /* Create the event flags group */
  if (tx_event_flags_create(&EventFlagsGroup, "Event Flags Group") != TX_SUCCESS)
  {
	ret = TX_GROUP_ERROR;
  }

  /* Create the event flags group */
  if (tx_event_flags_create(&SwarmDataReadyFlagsGroup, "Swarm Data Ready Event Flags Group") != TX_SUCCESS)
  {
	ret = TX_GROUP_ERROR;
  }

  /* Create the swarm mutex */
  if (tx_mutex_create(&SwarmMutex, "Swarm Data Mutex", TX_NO_INHERIT) != TX_SUCCESS)
  {
	ret = TX_MUTEX_ERROR;
  }

  /* Create GPSMutex.  */
  if (tx_mutex_create(&GPSMutex, "GPS Data Mutex", TX_NO_INHERIT) != TX_SUCCESS)
  {
	ret = TX_MUTEX_ERROR;
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
  {"help",	"print this message",	help_cmd},
  {"quit",	"quit",					quit_cmd},
  {"swarm",	"swarm satellite",		swarm_cmd},
//  {"led", "led test",           led_cmd},
  {"gps", 	"gps info",           	gps_cmd},
  {  NULL,	NULL,	NULL}
};

void ProcessSwarmCommands(ARG_LIST args)
{
	void (*cbFunc)(uint8_t *, uint16_t);
	uint8_t response[500] = {'\0'};
	S_SWARM_MSG msg, *qMsg;
//	osStatus_t status;
	uint32_t waitEventFlags = 0;
	ULONG flagVal;
	UINT flagStatus;
	uint32_t multiplicity = 1;	// default 1
	uint32_t repeatFlag = 0;

	memset(&msg, 0, sizeof(msg));

	// Cache current index value; when it changes then we know a key has been pressed
	int keyPressedFlag = uart_write_idx;

	if (args.count == 1)
	{
		args.count++;
		strcpy((char*)args.arg[1], "?");
	}

	if ((args.count == 3) && !strcmp((const char*)args.arg[2], "$"))
	{
		repeatFlag = 1;
		args.count--;
		strcpy((char *)args.arg[2],"");
	}

	if (args.count > 1)
	{
		do {
			if (SWARM_OK == swarm.DebugInterface(response, sizeof(response), args, msg.data, &cbFunc, &waitEventFlags, &multiplicity))
			{
				if (strlen((const char*)&msg.data) > 0)
				{
					qMsg = &msg;
					if (tx_queue_send(&SwarmMsgQueue, &qMsg, TX_WAIT_FOREVER) != TX_SUCCESS)
					{
						// OS Error trying to write to swarm
						printf("%s", MESSAGES[ERR_OCCURRED_MSG]);
						return;
					}
				}
				if (strlen((const char*)response) > 0)
				{
					printf("%s", response);
				}
				if (cbFunc)
				{
					if (waitEventFlags > 0)
					{
						while (multiplicity)	// Some messages contain multiple pieces of data
						{
							multiplicity--;
							flagVal = 0;
						#ifdef AZURE_RTOS_THREADX
							flagStatus = tx_event_flags_get(&SwarmDataReadyFlagsGroup, waitEventFlags, TX_AND_CLEAR, &flagVal, 150);
						#else
							flagVal = osEventFlagsWait(SwarmDataReadyFlagsHandle, waitEventFlags, osFlagsNoClear, 1000);
						#endif

							if ((flagVal & waitEventFlags) == waitEventFlags)
							{
//								HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

								#ifndef AZURE_RTOS_THREADX
									osEventFlagsClear(SwarmDataReadyFlagsHandle, waitEventFlags);
								#endif

								// Get response now that it is ready
								cbFunc(response,sizeof(response));
							}
							#ifdef AZURE_RTOS_THREADX
							else if ( flagStatus == TX_NO_EVENTS)
							{
								strcpy((char *)response, (const char*)MESSAGES[ERR_NO_RESP_MSG]);
							}
							else if ( flagStatus == TX_WAIT_ABORTED ||
												flagStatus == TX_WAIT_ERROR
								)
							#else
							else if (	flagVal == osFlagsErrorUnknown ||
												flagVal == osFlagsErrorTimeout ||
												flagVal == osFlagsErrorResource ||
												flagVal == osFlagsErrorParameter)
							#endif
							{
								strcpy((char *)response, (const char*)MESSAGES[ERR_OCCURRED_MSG]);
							}
							else
							{
								strcpy((char *)response, (const char*)MESSAGES[ERR_UNKNOWN_MSG]);
							}

							if (strlen((const char*)response) > 0)
							{
								printf("%s", response);
								#ifndef AZURE_RTOS_THREADX
									USB_TX_Completed = 0;
									while(USBD_OK != CDC_Transmit_FS(response, strlen((const char *)response)))
										;
									while (!USB_TX_Completed);
								#endif
							}
						}
					}
				}
			}
			else
			{
				// No valid commands found
				printf("%s", MESSAGES[ERR_CMD_NOT_FOUND_MSG]);
			}

			if (repeatFlag == 1)
			{
				memset(&msg, 0, sizeof(msg));
				HAL_Delay(1000);
			}
		}	while ((keyPressedFlag == uart_write_idx) && (repeatFlag == 1));	// loop forever if activated

		return;
	}
	// No valid commands found
	#ifdef AZURE_RTOS_THREADX
		printf("%s", MESSAGES[ERR_CMD_NOT_FOUND_MSG]);
	#else
		while(USBD_OK != CDC_Transmit_FS(MESSAGES[ERR_CMD_NOT_FOUND_MSG], strlen((const char *)MESSAGES[ERR_CMD_NOT_FOUND_MSG])));
	#endif
}


/* USER CODE BEGIN Header_StartStartupTask */
/**
  * @brief  Function implementing the startupTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStartupTask */
void StartupTask_Entry(ULONG thread_input)
{

	//
	// Initialize GPS module and DMA UART
	//
	GPS_Initialize(&gps, &GPSMutex);

	HAL_UARTEx_ReceiveToIdle_DMA(&gps_uart, uart3Buf, SM_BUF_SZ);
	__HAL_DMA_DISABLE_IT(&gps_dma_rx, DMA_IT_HT);

  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	/* Sleep for 500 ms */
	tx_thread_sleep(250);
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


/**
  * @brief  Function implementing the SwarmTask thread.
  * @param  thread_input: Not used
  * @retval None
  */
void SwarmTask_Entry(ULONG thread_input)
{
  UNUSED(thread_input);

  S_SWARM_MSG msg;
  S_SWARM_MSG *qMsg = NULL;


  memset(&msg, 0, sizeof(msg));

  SWARM_Initialize(&swarm, &SwarmMutex);
  HAL_UARTEx_ReceiveToIdle_DMA(&swarm_uart, uart2Buf, SM_BUF_SZ);
  __HAL_DMA_DISABLE_IT(&swarm_dma_rx, DMA_IT_HT);

  while (swarm.Startup((uint8_t*)&msg.data) < SWARM_IF_READY)
  {
	  SWARM_TX_Completed = 0;

	  while(HAL_OK != HAL_UART_Transmit_DMA(&swarm_uart, (uint8_t*)msg.data, strlen((const char*)msg.data)))
		  ;

	  while(!SWARM_TX_Completed);
	  memset(&msg, 0, sizeof(msg));
  }

  /* Infinite loop */
  while (1)
  {
	  if (tx_queue_receive(&SwarmMsgQueue, &qMsg, TX_WAIT_FOREVER) == TX_SUCCESS)
	  {
		  if (strlen((const char*)qMsg->data) > 0)
		  {
			while(HAL_OK != HAL_UART_Transmit_DMA(&swarm_uart, (uint8_t*)qMsg->data, strlen((const char*)qMsg->data)))
				;
		  }

	//			/* Toggle LED to indicate status*/
	//			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	//			/* Sleep for 500 ms */
	//			tx_thread_sleep(500);
	//			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &swarm_uart)
	{
		SWARM_TX_Completed = 1;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t swarm_buf_remainder;
	uint8_t gps_buf_remainder;

//	HAL_GPIO_TogglePin(GPIOG, TICK_TOGGLE_Pin);
	if (huart->Instance == SWARM_UART)
	{
		swarmBufCurrPos = swarmBufLastPos;

		if (swarmBufCurrPos + Size > LG_BUF_SZ)
		{
			swarm_buf_remainder = LG_BUF_SZ - swarmBufCurrPos;
			memcpy((uint8_t*)(swarmBuf+swarmBufCurrPos), uart2Buf, swarm_buf_remainder);
			swarm.ParseBuffer(&swarmBuf[swarmBufCurrPos], swarm_buf_remainder);

			swarmBufLastPos = Size - swarm_buf_remainder;
			memcpy((uint8_t*)swarmBuf, (uint8_t*)(uart2Buf+swarm_buf_remainder), swarmBufLastPos);
			swarm.ParseBuffer(&swarmBuf[0], swarmBufLastPos);
		}
		else
		{
			memcpy((uint8_t*)(swarmBuf+swarmBufCurrPos), uart2Buf, Size);
			swarmBufLastPos = Size + swarmBufCurrPos;
			swarm.ParseBuffer(&swarmBuf[swarmBufCurrPos], Size);
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&swarm_uart, uart2Buf, SM_BUF_SZ);
		__HAL_DMA_DISABLE_IT(&swarm_dma_rx, DMA_IT_HT);
//	HAL_GPIO_TogglePin(GPIOG, TICK_TOGGLE_Pin);
	}

	//	HAL_GPIO_TogglePin(GPIOG, TICK_TOGGLE_Pin);
	if (huart->Instance == GPS_UART)
	{
		gpsBufCurrPos = gpsBufLastPos;

		if (gpsBufCurrPos + Size > LG_BUF_SZ)
		{
			gps_buf_remainder = LG_BUF_SZ - gpsBufCurrPos;
			memcpy((uint8_t*)(gpsBuf+gpsBufCurrPos), uart3Buf, gps_buf_remainder);
			gps.ParseBuffer(&gpsBuf[gpsBufCurrPos], gps_buf_remainder);

			gpsBufLastPos = Size - gps_buf_remainder;
			memcpy((uint8_t*)gpsBuf, (uint8_t*)(uart3Buf+gps_buf_remainder), gpsBufLastPos);
			gps.ParseBuffer(&gpsBuf[0], gpsBufLastPos);
		}
		else
		{
			memcpy((uint8_t*)(gpsBuf+gpsBufCurrPos), uart3Buf, Size);
			gpsBufLastPos = Size + gpsBufCurrPos;
			gps.ParseBuffer(&gpsBuf[gpsBufCurrPos], Size);
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&gps_uart, uart3Buf, SM_BUF_SZ);
		__HAL_DMA_DISABLE_IT(&gps_dma_rx, DMA_IT_HT);
//	HAL_GPIO_TogglePin(GPIOG, TICK_TOGGLE_Pin);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

	if (huart->Instance == SWARM_UART)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, uart2Buf, SM_BUF_SZ);
	}
	else if (huart->Instance == GPS_UART)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3Buf, SM_BUF_SZ);
	}

}

/* USER CODE END 1 */

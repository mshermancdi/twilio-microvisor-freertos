/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.h
  * @author  MCD Application Team
  * @brief   ThreadX applicative header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_THREADX_H
#define __APP_THREADX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../../threadx/common/inc/tx_api.h"
#include "debug_utils.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);
void MX_ThreadX_Init(void);
void ProcessSwarmCommands(ARG_LIST args);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_STACK_SIZE						1024
#define STARTUP_STACK_SIZE                  3*APP_STACK_SIZE
#define MAG_DATA_AQ_STACK_SIZE              4*APP_STACK_SIZE
#define DSP_STACK_SIZE                      28*APP_STACK_SIZE
#define TRIGGER_DETECT_STACK_SIZE           28*APP_STACK_SIZE
#define PASS_VERIFY_STACK_SIZE              46*APP_STACK_SIZE
#define SERVICE_PORT_STACK_SIZE				4*APP_STACK_SIZE
#define SWARM_STACK_SIZE						2*APP_STACK_SIZE
#define USER_BUTTON_STACK_SIZE				APP_STACK_SIZE
#define LED_TASK_STACK_SIZE					APP_STACK_SIZE

#define SWARM_QUEUE_SIZE						5
//
//	Define priorities  - Lower value --> Higher priority
//
#define THREAD_STARTUP_PRIO                     			10
#define THREAD_STARTUP_PREEMPTION_THRESHOLD					THREAD_STARTUP_PRIO

#define THREAD_DSP_PRIO                         			12
#define THREAD_DSP_PREEMPTION_THRESHOLD         			THREAD_DSP_PRIO

#define THREAD_MAG_DATA_AQ_PRIO                 			11
#define THREAD_MAG_DATA_AQ_PREEMPTION_THRESHOLD   			THREAD_MAG_DATA_AQ_PRIO

#define THREAD_TRIGGER_DETECT_PRIO               			13
#define THREAD_TRIGGER_DETECT_PREEMPTION_THRESHOLD			THREAD_TRIGGER_DETECT_PRIO

#define THREAD_PASS_VERIFY_PRIO1               				20
#define THREAD_PASS_VERIFY_PREEMPTION_THRESHOLD1			THREAD_PASS_VERIFY_PRIO1

#define THREAD_PASS_VERIFY_PRIO2               				21
#define THREAD_PASS_VERIFY_PREEMPTION_THRESHOLD2			THREAD_PASS_VERIFY_PRIO2

#define USER_BUTTON_TASK_PRIO               				26
#define USER_BUTTON_TASK_PREEMPTION_THRESHOLD				USER_BUTTON_TASK_PRIO

#define SWARM_TASK_PRIO                  					27
#define SWARM_TASK_PREEMPTION_THRESHOLD						SWARM_TASK_PRIO

#define LED_TASK_PRIO               						28
#define LED_TASK_PREEMPTION_THRESHOLD						LED_TASK_PRIO

#define THREAD_SERVICE_PORT_PRIO               				31
#define THREAD_SERVICE_PORT_PREEMPTION_THRESHOLD   			THREAD_SERVICE_PORT_PRIO
/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */

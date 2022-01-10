/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpdma.c
  * @brief   This file provides code for the configuration
  *          of the GPDMA instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "gpdma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

DMA_HandleTypeDef handle_GPDMA1_Channel4;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel12;
DMA_HandleTypeDef handle_GPDMA1_Channel12;
DMA_HandleTypeDef handle_GPDMA1_Channel4;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* GPDMA1 init function */
void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */
/* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  /* USER CODE END GPDMA1_Init 0 */

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  handle_GPDMA1_Channel4.Instance = GPDMA1_Channel4;
  handle_GPDMA1_Channel4.Init.Request = DMA_REQUEST_SW;
  handle_GPDMA1_Channel4.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  handle_GPDMA1_Channel4.Init.Direction = DMA_MEMORY_TO_MEMORY;
  handle_GPDMA1_Channel4.Init.SrcInc = DMA_SINC_INCREMENTED;
  handle_GPDMA1_Channel4.Init.DestInc = DMA_DINC_INCREMENTED;
  handle_GPDMA1_Channel4.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel4.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel4.Init.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel4.Init.SrcBurstLength = 1;
  handle_GPDMA1_Channel4.Init.DestBurstLength = 1;
  handle_GPDMA1_Channel4.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  handle_GPDMA1_Channel4.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel4.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&handle_GPDMA1_Channel4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel4, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel3.Instance = GPDMA1_Channel3;
  handle_GPDMA1_Channel3.Init.Request = DMA_REQUEST_SW;
  handle_GPDMA1_Channel3.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  handle_GPDMA1_Channel3.Init.Direction = DMA_MEMORY_TO_MEMORY;
  handle_GPDMA1_Channel3.Init.SrcInc = DMA_SINC_INCREMENTED;
  handle_GPDMA1_Channel3.Init.DestInc = DMA_DINC_INCREMENTED;
  handle_GPDMA1_Channel3.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel3.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel3.Init.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel3.Init.SrcBurstLength = 1;
  handle_GPDMA1_Channel3.Init.DestBurstLength = 1;
  handle_GPDMA1_Channel3.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  handle_GPDMA1_Channel3.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel3.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&handle_GPDMA1_Channel3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel3, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel2.Instance = GPDMA1_Channel2;
  handle_GPDMA1_Channel2.Init.Request = DMA_REQUEST_SW;
  handle_GPDMA1_Channel2.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  handle_GPDMA1_Channel2.Init.Direction = DMA_MEMORY_TO_MEMORY;
  handle_GPDMA1_Channel2.Init.SrcInc = DMA_SINC_INCREMENTED;
  handle_GPDMA1_Channel2.Init.DestInc = DMA_DINC_INCREMENTED;
  handle_GPDMA1_Channel2.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel2.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel2.Init.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel2.Init.SrcBurstLength = 1;
  handle_GPDMA1_Channel2.Init.DestBurstLength = 1;
  handle_GPDMA1_Channel2.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  handle_GPDMA1_Channel2.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel2.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&handle_GPDMA1_Channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel2, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel1.Instance = GPDMA1_Channel1;
  handle_GPDMA1_Channel1.Init.Request = DMA_REQUEST_SW;
  handle_GPDMA1_Channel1.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  handle_GPDMA1_Channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  handle_GPDMA1_Channel1.Init.SrcInc = DMA_SINC_INCREMENTED;
  handle_GPDMA1_Channel1.Init.DestInc = DMA_DINC_INCREMENTED;
  handle_GPDMA1_Channel1.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  handle_GPDMA1_Channel1.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  handle_GPDMA1_Channel1.Init.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel1.Init.SrcBurstLength = 1;
  handle_GPDMA1_Channel1.Init.DestBurstLength = 1;
  handle_GPDMA1_Channel1.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  handle_GPDMA1_Channel1.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel1.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&handle_GPDMA1_Channel1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
  handle_GPDMA1_Channel0.Init.Request = DMA_REQUEST_SW;
  handle_GPDMA1_Channel0.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  handle_GPDMA1_Channel0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  handle_GPDMA1_Channel0.Init.SrcInc = DMA_SINC_INCREMENTED;
  handle_GPDMA1_Channel0.Init.DestInc = DMA_DINC_INCREMENTED;
  handle_GPDMA1_Channel0.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel0.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
  handle_GPDMA1_Channel0.Init.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel0.Init.SrcBurstLength = 1;
  handle_GPDMA1_Channel0.Init.DestBurstLength = 1;
  handle_GPDMA1_Channel0.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  handle_GPDMA1_Channel0.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel0.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&handle_GPDMA1_Channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel12.Instance = GPDMA1_Channel12;
  handle_GPDMA1_Channel12.InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel12.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel12.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_GPDMA1_Channel12.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  handle_GPDMA1_Channel12.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel12) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel12, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GPDMA1_Init 2 */
	
	/* GPDMA interrupt Init */
	HAL_NVIC_SetPriority(GPDMA1_Channel12_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel12_IRQn);
	
	HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

	HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

	HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
	
	HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
	
	HAL_NVIC_SetPriority(GPDMA1_Channel4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);	
	
	HAL_NVIC_SetPriority(GPDMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);	
	
	HAL_NVIC_SetPriority(GPDMA1_Channel6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel6_IRQn);	
	
	HAL_NVIC_SetPriority(GPDMA1_Channel7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel7_IRQn);	
	
  /* USER CODE END GPDMA1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

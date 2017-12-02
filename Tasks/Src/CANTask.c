/**
  ******************************************************************************
  * File Name          : CANTask.c
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])
uint8_t isRcanStarted_CMGM = 0;
CanRxMsgTypeDef CMGMCanRxMsg;
Motor820RRxMsg_t CMFLRx,CMFRRx;
Motor6623RxMsg_t GMPITCHRx,GMYAWRx;
/********************CAN******************************/
void InitCanReception()
{
	//http://www.eeworld.com.cn/mcu/article_2016122732674_3.html
	CMGMMOTOR_CAN.pRxMsg = &CMGMCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&CMGMMOTOR_CAN, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcanStarted_CMGM = 1;
}

//CAN接收中断入口函数
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	//处理CAN接收数据前关中断
	HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
	HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
	if(hcan == &CMGMMOTOR_CAN){//CAN1数据
		switch(CMGMCanRxMsg.StdId){
			case CMFL_RXID:
				CMFLRx.angle = CanRxGetU16(CMGMCanRxMsg, 0);
				CMFLRx.RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				break;
			case CMFR_RXID:
				CMFRRx.angle = CanRxGetU16(CMGMCanRxMsg, 0);
				CMFRRx.RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				break;
			case GMYAW_RXID:
				GMYAWRx.angle = CanRxGetU16(CMGMCanRxMsg, 0);
				GMYAWRx.realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				GMYAWRx.giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				break;
			case GMPITCH_RXID:
				GMPITCHRx.angle = CanRxGetU16(CMGMCanRxMsg, 0);
				GMPITCHRx.realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				GMPITCHRx.giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				break;
			default:
			Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
			isRcanStarted_CMGM = 0;
		}else{
			isRcanStarted_CMGM = 1;
		}
	}
	//处理CAN接收数据后开中断
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
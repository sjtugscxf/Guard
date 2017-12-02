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
uint8_t isRcanStarted_CMGM = 0, isRcanStarted_ZGYRO = 0;
CanRxMsgTypeDef CMGMCanRxMsg, ZGYROCanRxMsg;
Motor820RRxMsg_t CMFLRx,CMFRRx,CMBLRx,CMBRRx;
Motor6623RxMsg_t GMPITCHRx,GMYAWRx;
float ZGyroModuleAngle = 0.0;
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
	
	ZGYRO_CAN.pRxMsg = &ZGYROCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = 0;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.BankNumber = 14;
  HAL_CAN_ConfigFilter(&ZGYRO_CAN, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcanStarted_ZGYRO = 1;
}

//CAN接收中断入口函数
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	//处理CAN接收数据前关中断
	HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
	HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
	HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
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
			case CMBL_RXID:
				CMBLRx.angle = CanRxGetU16(CMGMCanRxMsg, 0);
				CMBLRx.RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				break;
			case CMBR_RXID:
				CMBRRx.angle = CanRxGetU16(CMGMCanRxMsg, 0);
				CMBRRx.RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
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
	else if(hcan == &ZGYRO_CAN)//CAN2数据
	{
		switch(ZGYROCanRxMsg.StdId)
		{
			case ZGYRO_RXID:
			 {
				CanRxMsgTypeDef *msg = &ZGYROCanRxMsg;
				ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
			 }
			 break;
			default:
			Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK)
		{
			isRcanStarted_ZGYRO = 0;
		}else{
			isRcanStarted_ZGYRO = 1;
		}
	}
	//处理CAN接收数据后开中断
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

//单轴陀螺仪初始化，在主控制任务中，开机三秒后执行
void GYRO_RST(void)
{
	CanTxMsgTypeDef pData;
	ZGYRO_CAN.pTxMsg = &pData;
	
	ZGYRO_CAN.pTxMsg->StdId = ZGYRO_TXID;
	ZGYRO_CAN.pTxMsg->ExtId = 0;
	ZGYRO_CAN.pTxMsg->IDE = CAN_ID_STD;
	ZGYRO_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	ZGYRO_CAN.pTxMsg->DLC = 0x08;
	ZGYRO_CAN.pTxMsg->Data[0] = 0x00;
	ZGYRO_CAN.pTxMsg->Data[1] = 0x01;
	ZGYRO_CAN.pTxMsg->Data[2] = 0x02;
	ZGYRO_CAN.pTxMsg->Data[3] = 0x03;
	ZGYRO_CAN.pTxMsg->Data[4] = 0x04;
	ZGYRO_CAN.pTxMsg->Data[5] = 0x05;
	ZGYRO_CAN.pTxMsg->Data[6] = 0x06;
	ZGYRO_CAN.pTxMsg->Data[7] = 0x07;

	HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
	if(HAL_CAN_Transmit_IT(&ZGYRO_CAN) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
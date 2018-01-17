/**
  ******************************************************************************
  * File Name          : CANTask.h
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CANTASK_H
#define __CANTASK_H

#include "includes.h"

#define CMGMMOTOR_CAN hcan1

//RxID
#define CMFL_RXID 0x201u
#define CMFR_RXID 0x202u
#define BULLET_RXID 0x203u
#define BULLET2_RXID 0x204u

#define GMYAW_RXID 0x206u
#define GMPITCH_RXID 0x205u

//TxID
#define CM_TXID 0x200u
#define GM_TXID 0x1FFu

typedef struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
}Motor820RRxMsg_t;

typedef struct{
	uint16_t angle;
	int16_t realIntensity;
	int16_t giveIntensity;
}Motor6623RxMsg_t;

extern Motor820RRxMsg_t CMFLRx;
extern Motor820RRxMsg_t CMFRRx;
extern Motor820RRxMsg_t BulletRx;
extern Motor820RRxMsg_t Bullet2Rx;
extern Motor6623RxMsg_t GMPITCHRx;
extern Motor6623RxMsg_t	GMYAWRx;

extern uint8_t can1_update;
extern uint8_t can1_type;

void InitCanReception();

#endif /*__ CANTASK_H */

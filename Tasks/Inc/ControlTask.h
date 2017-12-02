/**
  ******************************************************************************
  * File Name          : ControlTask.h
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "includes.h"

#define YAW_OFFSET         8000u  
#define PITCH_OFFSET       3500u  
#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	1.4f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	6.5f,\
	0.0f,\
	1.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

typedef enum
{
	PREPARE_STATE,     	
	NORMAL_STATE,		  
  DEFEND_STATE,
  ATTACK_STATE,  
	STOP_STATE        
}WorkState_e;

extern WorkState_e WorkState;
extern uint8_t find_enemy;
extern uint16_t enemy_pitch;
extern uint16_t enemy_yaw;
extern uint16_t enemy_detect_cnt;

void CMControlInit(void);

#endif /*__ CONTROLTASK_H */

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

#define AUTO_ATTACK_YAW_KP      0.005f
#define AUTO_ATTACK_YAW_KD      0 
#define AUTO_ATTACK_PITCH_KP      0.005f
#define AUTO_ATTACK_PITCH_KD      0 
#define YAW_OFFSET         330u  
#define PITCH_OFFSET       370u  
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

#define BULLET_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	10000,\
	10000,\
	10000,\
	0,\
	10000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define BULLET_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	1.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	10000,\
	10000,\
	10000,\
	0,\
	4900,\
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

extern int16_t yawIntensity ;

extern uint16_t maincnt;

extern fw_PID_Regulator_t pitchPositionPID;
extern fw_PID_Regulator_t yawPositionPID;
extern fw_PID_Regulator_t pitchSpeedPID;
extern fw_PID_Regulator_t yawSpeedPID;

extern double bullet_angle_target;
extern float yawRealAngle;
extern float pitchRealAngle;

void CMControlInit(void);

#endif /*__ CONTROLTASK_H */

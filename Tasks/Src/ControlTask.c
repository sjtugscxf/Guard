/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_GREEN_OFF()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_RED_OFF()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET)
#define LED_GREEN_ON()    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_RED_ON()      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET)

uint8_t find_enemy = 0;
uint16_t enemy_yaw = YAW_OFFSET;
uint16_t enemy_pitch = PITCH_OFFSET;
uint16_t enemy_detect_cnt = 0;
WorkState_e WorkState = PREPARE_STATE;
uint16_t prepare_time = 0;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t BulletSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

int16_t CMFLIntensity = 0, CMFRIntensity = 0, BulletIntensity = 0;
int16_t yawIntensity = 0;		
int16_t pitchIntensity = 0;

//底盘PID初始化
void CMControlInit(void)
{
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	BulletSpeedPID.Reset(&BulletSpeedPID);
}

//单个底盘电机的控制，下同
void ControlCMFL(void)
{		
	CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075;
	CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;	
			
	CM1SpeedPID.fdb = CMFLRx.RotateSpeed;

	CM1SpeedPID.Calc(&CM1SpeedPID);
	CMFLIntensity = CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output;
}

void ControlCMFR(void)
{		
	CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075;
	CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;	
			
	CM2SpeedPID.fdb = CMFRRx.RotateSpeed;

	CM2SpeedPID.Calc(&CM2SpeedPID);
	CMFRIntensity = CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output;
}

void ControlBullet(void)
{		
	BulletSpeedPID.ref = bullet_ref*0.075;
	BulletSpeedPID.ref = 160 * BulletSpeedPID.ref;	
			
	BulletSpeedPID.fdb = BulletRx.RotateSpeed;

	BulletSpeedPID.Calc(&BulletSpeedPID);
	BulletIntensity = CHASSIS_SPEED_ATTENUATION * BulletSpeedPID.output;
}

//状态机切换
void WorkStateFSM(void)
{
	static int blink_cnt = 0;
	blink_cnt++;
	switch (WorkState)
	{
		case PREPARE_STATE:
		{
			if(prepare_time<2000) prepare_time++;
			if(prepare_time == 2000)//开机两秒进入正常模式
			{
				WorkState = NORMAL_STATE;
				prepare_time = 0;
				LED_GREEN_OFF();
				LED_RED_OFF();
				blink_cnt = 0;
				printf("START\n");
			}
			
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case NORMAL_STATE://正常遥控调试模式
		{
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else if (inputmode == AUTO)
			{
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					WorkState = DEFEND_STATE;//防御模式开启摩擦轮
				}
			}
			
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				LED_GREEN_TOGGLE();
				printf("%d %d \n",enemy_yaw,enemy_pitch);
			}
		}break;
		case DEFEND_STATE:  //防御模式，云台360度旋转
		{
			if (find_enemy == 1) WorkState = ATTACK_STATE;
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else if (inputmode == REMOTE_INPUT)
			{
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				WorkState = NORMAL_STATE;
			}
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				LED_RED_TOGGLE();
			}
		}break;
		case ATTACK_STATE:  //自动打击模式
		{
			if (find_enemy == 0) WorkState = DEFEND_STATE;
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else if (inputmode == REMOTE_INPUT)
			{
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				WorkState = NORMAL_STATE;
			}
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				LED_GREEN_TOGGLE();
				LED_RED_TOGGLE();
			}
		}break;
		case STOP_STATE://紧急停止
		{
			if (inputmode == REMOTE_INPUT)
			{
				WorkState = PREPARE_STATE;
				RemoteTaskInit();
			}
		}break;
	}
}

//底盘电机CAN信号控制
void setCMMotor()
{
	CanTxMsgTypeDef pData;
	CMGMMOTOR_CAN.pTxMsg = &pData;
	
	CMGMMOTOR_CAN.pTxMsg->StdId = CM_TXID;
	CMGMMOTOR_CAN.pTxMsg->ExtId = 0;
	CMGMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	CMGMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CMGMMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	CMGMMOTOR_CAN.pTxMsg->Data[0] = 0x00;
	CMGMMOTOR_CAN.pTxMsg->Data[1] = 0x00;
	CMGMMOTOR_CAN.pTxMsg->Data[2] = 0x00;
	CMGMMOTOR_CAN.pTxMsg->Data[3] = 0x00;
	CMGMMOTOR_CAN.pTxMsg->Data[4] = (uint8_t)(BulletIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[5] = (uint8_t)BulletIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[6] = 0x00;
	CMGMMOTOR_CAN.pTxMsg->Data[7] = 0x00;

	if(can1_update == 1 && can1_type == 0)
	{
		//CAN通信前关中断
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		//CAN通信后开中断，防止中断影响CAN信号发送
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}

//云台电机CAN信号控制
void setGMMotor()
{
	CanTxMsgTypeDef pData;
	CMGMMOTOR_CAN.pTxMsg = &pData;
	
	CMGMMOTOR_CAN.pTxMsg->StdId = GM_TXID;
	CMGMMOTOR_CAN.pTxMsg->ExtId = 0;
	CMGMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	CMGMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CMGMMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	CMGMMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(pitchIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)pitchIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(yawIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)yawIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[4] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[5] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[6] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}

#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(5.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(6.5, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 3500.0);
//fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(30.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(10.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 2000.0);
#define yaw_zero 7200  //100
#define pitch_zero 4600
float yawRealAngle = 0.0;
float pitchRealAngle = 0.0;
float gap_angle = 0.0;

//控制云台YAW轴
void ControlYawSpeed(void)
{	
	yawIntensity = ProcessYawPID(yawSpeedTarget,-gYroZs);
}

//控制云台pitch轴
void ControlPitch(void)
{
	uint16_t pitchZeroAngle = pitch_zero;
				
	pitchRealAngle = -(GMPITCHRx.angle - pitchZeroAngle) * 360 / 8192.0;
	NORMALIZE_ANGLE180(pitchRealAngle);

	MINMAX(pitchAngleTarget, -25.0f, 13);
				
	pitchIntensity = ProcessPitchPID(pitchAngleTarget,pitchRealAngle,gYroYs);
}

//主控制循环
void controlLoop()
{
	if(enemy_detect_cnt>1000)    //2s内没有刷新自动打击数据则回中
	{
		enemy_yaw = YAW_OFFSET;
		enemy_pitch = PITCH_OFFSET;
	}
	else
	{
		enemy_detect_cnt++;
	}
	
	WorkStateFSM();
	
	if(WorkState == DEFEND_STATE)
	{
		yawSpeedTarget = 220.0;
	}
	
	if(WorkState == ATTACK_STATE)
	{
		static float enemy_yaw_err_last = 0;
		float enemy_yaw_err = (float)((int16_t)YAW_OFFSET - enemy_yaw);
		float enemy_yaw_out = enemy_yaw_err/1000 * fabs(enemy_yaw_err)  * AUTO_ATTACK_YAW_KP + (enemy_yaw_err - enemy_yaw_err_last)*AUTO_ATTACK_YAW_KD;
		if (enemy_yaw_out>2) enemy_yaw_out = 2;
		else if (enemy_yaw_out<-2) enemy_yaw_out = -2;
		yawSpeedTarget = enemy_yaw_out;
		
		static float enemy_pitch_err_last = 0;
		float enemy_pitch_err = (float)((int16_t)PITCH_OFFSET - enemy_pitch);
		float enemy_pitch_out = enemy_pitch_err/1000 * fabs(enemy_pitch_err) * AUTO_ATTACK_PITCH_KP + (enemy_pitch_err - enemy_pitch_err_last)*AUTO_ATTACK_PITCH_KD;
		if (enemy_pitch_out>1) enemy_pitch_out = 1;
		else if (enemy_pitch_out<-1) enemy_pitch_out = -1;
		pitchAngleTarget -= enemy_pitch_out;
	}
	
	if(WorkState != STOP_STATE) 
	{
		ControlYawSpeed();
		ControlPitch();
		setGMMotor();
		
		//ControlCMFL();
		//ControlCMFR();
		//ControlBullet();
		//setCMMotor();
	}
}

//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		//主循环在时间中断中启动
		controlLoop();
	}
	else if (htim->Instance == htim7.Instance)
	{
		rc_cnt++;
		if (rc_update)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
		    HAL_UART_AbortReceive(&RC_UART);
		    HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame) WorkState = PREPARE_STATE;
				HAL_UART_AbortReceive(&RC_UART);
		    HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
				rc_first_frame = 1;
			}
			rc_update = 0;
		}
	}
}
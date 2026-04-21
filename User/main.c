#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

int16_t AX, AY, AZ, GX, GY, GZ;
uint8_t TimerErrorFlag;
uint16_t TimerCount;
uint8_t TiltProtectFlag = 0;

float AngleAcc;
float AngleGyro;
float Angle;

int16_t LeftPWM,RightPWM;
int16_t AvePWM,DifPWM;

float LeftSpeed,RightSpeed;
float AveSpeed,DifSpeed;

PID_t AnglePID = {
	.Kp=4.5,
	.Ki=0.1,
	.Kd=4.5,
	
	.OutMax=80,
	.OutMin=-80,
	
	.OutOffset = 4,
	
	.ErrorIntMax = 500,
	.ErrorIntMin = -500,
};

PID_t SpeedPID = {
	.Kp=2.5,
	.Ki=0.08,
	.Kd=0,
	
	.OutMax=20,
	.OutMin=-20,
	
	.ErrorIntMax = 150,//150
	.ErrorIntMin = -150,
};

//PID_t TurnPID = {
//	.Kp=4,
//	.Ki=3,
//	.Kd=0,
//	
//	.OutMax=40,
//	.OutMin=-40,

//	.ErrorIntMax = 20,
//	.ErrorIntMin = -20,
//};

int main(void)
{
	OLED_Init();
	MPU6050_Init();
	BlueSerial_Init();
	LED_Init();
	Motor_Init();
	Encoder_Init();
	
	Timer_Init();
	
//	PID_Init(&AnglePID); 
//	PID_Init(&SpeedPID);
	
	while (1)
	{
		
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);//+2.6
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);//+32
		
		OLED_Printf(50, 0, OLED_6X8, "  Speed");
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
		OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);
		OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);//+2.6
		OLED_Printf(50, 48, OLED_6X8, "%+05.0f", SpeedPID.Out);
		
		
//		OLED_Printf(88, 0, OLED_6X8, " Turn");
//		OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
//		OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
//		OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
//		OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);
//		OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);//
//		OLED_Printf(88, 48, OLED_6X8, "%+05.0f", TurnPID.Out);
//		OLED_Printf(56, 56, OLED_6X8, "Os:%02.0f", AnglePID.OutOffset);//+32
		OLED_Update();
		
		
		if (BlueSerial_RxFlag == 1)
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
			}
			else if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if (strcmp(Name, "AngleKp") == 0)
				{
					AnglePID.Kp = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					AnglePID.Ki = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{
					AnglePID.Kd = atof(Value);
				}
				if (strcmp(Name, "SpeedKp") == 0)
				{
					SpeedPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					SpeedPID.Ki = atof(Value);
				}
				else if (strcmp(Name, "SpeedKd") == 0)
				{
					SpeedPID.Kd = atof(Value);
				}
//				if (strcmp(Name, "TurnKp") == 0)
//				{
//					TurnPID.Kp = atof(Value);
//				}
//				else if (strcmp(Name, "TurnKi") == 0)
//				{
//					TurnPID.Ki = atof(Value);
//				}
//				else if (strcmp(Name, "TurnKd") == 0)
//				{
//					TurnPID.Kd = atof(Value);
//				}
				else if (strcmp(Name, "Offset") == 0)
				{
					AnglePID.OutOffset = atof(Value);
				}
			}
			else if (strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				SpeedPID.Target = LV / 25.0;
				DifPWM = -RH * 0.8f;
				
				//DifPWM = RH / 25.0;
				
//				SpeedPID.Target = LV / 25.0;
//				TurnPID.Target = RH / 25.0;
			}
			
			BlueSerial_RxFlag = 0;
		}
		
		BlueSerial_Printf("[plot,%f,%f]", SpeedPID.Target, AveSpeed);
	}
}

void TIM4_IRQHandler(void)
{
	static uint16_t Count0,Count1;
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		
		Count0++;
		if(Count0 >= 10)
		{
			Count0=0;
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
			GY += 55;//55
			
			AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
			AngleAcc += 2.8;//2.6
			
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
			
			float Alpha = 0.01;
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
			
			if(Angle > 50 || Angle < -50)
			{
				Motor_SetPWM(1,0); 
				Motor_SetPWM(2,0); 
				
				PID_Init(&AnglePID); 
				PID_Init(&SpeedPID);
//				PID_Init(&TurnPID);
				
				return;
			}
			
			AnglePID.Actual = Angle;
			PID_Update(&AnglePID);
			AvePWM = -AnglePID.Out;
			
			LeftPWM = AvePWM + DifPWM / 2;
			RightPWM = AvePWM - DifPWM / 2;
			
			
			if(LeftPWM > 100) {LeftPWM = 100;} else if(LeftPWM < -100) {LeftPWM = -100;}
			if(RightPWM > 100) {RightPWM = 100;} else if(RightPWM < -100) {RightPWM = -100;}
			
			Motor_SetPWM(1,LeftPWM); 
			Motor_SetPWM(2,RightPWM);
			
		}
		
		Count1++;
		if(Count1 >= 50)
		{
			Count1=0;
			
			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;
			

			AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
			DifSpeed = LeftSpeed - RightSpeed;
			
			SpeedPID.Actual = AveSpeed;
			PID_Update(&SpeedPID);
			AnglePID.Target = SpeedPID.Out;
			
//			TurnPID.Actual = -DifSpeed;
//			PID_Update(&TurnPID);
//			DifPWM = TurnPID.Out;
			
		}
		
		
		if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
		{

			TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		}
	}
}

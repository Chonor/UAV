/******************** (C) COPYRIGHT 2015 FTC ***************************
 * ??		 :FTC
 * ???  :FTC_FlyControl.cpp
 * ??    :????
**********************************************************************************/
#include "FTC_FlyControl.h"

FTC_FlyControl fc;

FTC_FlyControl::FTC_FlyControl()
{
	rollPitchRate = 150;
	yawRate = 50;
	
	altHoldDeadband = 30;
	NowThr = LastThr = 1249;
	flag_return = flag_height = flag_fly = flag_throw = 0;
	Height = 500;
	//??PID??
	PID_Reset();
	will_return = 1;
	record[ROLL]=0;
	record[PITCH]=0;
}

//??PID??
void FTC_FlyControl::PID_Reset(void)
{
	pid[PIDROLL].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDPITCH].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDYAW].set_pid(0.8, 0.45, 0, 200);
	pid[PIDANGLE].set_pid(5, 0, 0, 0);
	pid[PIDMAG].set_pid(2, 0, 0, 0);
 	pid[PIDVELZ].set_pid(1.45, 0.5, 0.002, 150);
 	pid[PIDALT].set_pid(1.2, 0, 0, 200);
}

void FTC_FlyControl::Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_ADC = imu.Gyro_lpf / 4.0f;
	float roll_r, pitch_r;
	
	//????
	rc.Command[ROLL] = applyDeadband(rc.Command[ROLL], 25);
	rc.Command[PITCH] = applyDeadband(rc.Command[PITCH], 25);
	rc.Command[YAW] = applyDeadband(rc.Command[YAW], 25);
	
	if(!ftc.f.ARMED){
		record[ROLL]=0;
	  record[PITCH]=0;
	}
	//返航
	if(flag_return != 1) {
		errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 1.5) , -FLYANGLE_MAX, +FLYANGLE_MAX) 
			- imu.angle.x * 10; 
		errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 1.5) , -FLYANGLE_MAX, +FLYANGLE_MAX) 
			- imu.angle.y * 10;
	} else {
		//参数归一化，倾角<100度
		float norm = sqrt(record[PITCH] * record[PITCH] + record[ROLL] * record[ROLL]) / 100;
		if(norm < 0.8)
			norm = 0.8;
		//反向
		roll_r = -record[ROLL]/norm, pitch_r= -record[PITCH]/norm;
		errorAngle[ROLL] = constrain_int32((int32_t)roll_r , -100, +100) 
			- imu.angle.x * 10; 
		errorAngle[PITCH] = constrain_int32((int32_t)pitch_r , -100, +100) 
			- imu.angle.y * 10;
	}
	
	if(!flag_return){
		//积分摇杆
		record[ROLL] += rc.Command[ROLL] * 0.004f;
		record[PITCH] += rc.Command[PITCH] * 0.004f;
	}else{
		//回程时也积分，权值较小
		record[ROLL] += roll_r * 0.002f;
		record[PITCH] += pitch_r * 0.002f;
	}
	imu.Mag.x = record[PITCH];
	imu.Mag.y = record[ROLL];
	//????
	errorAngle[ROLL] = applyDeadband(errorAngle[ROLL], 2);
	errorAngle[PITCH] = applyDeadband(errorAngle[PITCH], 2);

	//??P
	RateError[ROLL] = pid[PIDANGLE].get_p(errorAngle[ROLL]) - Gyro_ADC.x;
	RateError[PITCH] = pid[PIDANGLE].get_p(errorAngle[PITCH]) - Gyro_ADC.y;
	RateError[YAW] = (((int32_t)(yawRate) * rc.Command[YAW]) >> 5) - Gyro_ADC.z;		
}

void FTC_FlyControl::Attitude_Inner_Loop(void)
{
	int32_t PIDTerm[3];
	static Vector3f mes(0, 0, ACC_1G);
	
	//
	//float tiltAngle = constrain_float(max(abs(imu.angle.x), abs(imu.angle.y)), 0 ,20);
	float tiltAngle = constrain_float(abs(mes.angle(imu.grav)), 0 , M_PI / 9.0f);
	
	//PID
	for(u8 i=0; i<3;i++)
	{
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)
			pid[i].reset_I();
		PIDTerm[i] = pid[i].get_pid(RateError[i], 0.002f);
	}
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300-abs(rc.Command[YAW]), +300+abs(rc.Command[YAW]));	
	

	if(autoUp > 0 && flag_fly == 1 && flag_throw == 1) {  //抛飞
			autoUp--;	
			if(autoUp <= 10){					//抛飞停止
				flag_fly = 0;
				autoUp = 0;
				Height = nav.ultraAlt;
			}
			LastThr = NowThr = 1600;
	}else{ //定高  缓降
		if(flag_height == 1 || ( flag_fly == 0 && flag_throw == 1 )) { //定高
			LastThr = NowThr = this->AltThr;
		} else if (flag_height == 2 || flag_throw == 2  || flag_return == 2){
			if(imu.Acc.z < 3930 && NowThr < 1800){  //缓降
				NowThr=LastThr + 5;
			}else if(NowThr > 1200){
				NowThr=LastThr - 7;
			}	
				LastThr = NowThr;
		}else{
			LastThr = NowThr = rc.Command[THROTTLE] - 1;
		}
	}
	//油门补偿
	if(!ftc.f.ALTHOLD && !flag_height){
		//NowThr = (NowThr - 1000) / cosf(radians(tiltAngle)) + 1000;
	  NowThr = (NowThr - 1000) / cosf(tiltAngle) + 1000;
	}
	NowThr = constrain_int32(NowThr, 1000, 1800);	
	
	motor.writeMotor(NowThr, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}

//高度控制外循环
void FTC_FlyControl::Altitude_Outter_Loop(void)
{
	if(flag_throw) this->AltHold= Height;
	else this->AltHold = rc.Command[THROTTLE] - 300;
	this->AltHold = constrain_int32(AltHold, 700, 1500);
	this->AltErr = this->AltHold - nav.ultraAlt;
	this->AltErr = applyDeadband(this->AltErr, altHoldDeadband);
	this->AltErr = constrain_int32(this->AltErr, -500, 600);	
	
	//????
	this->AltSpeedError = pid[PIDALT].get_p(this->AltErr) - nav.ultraVel;
}

//高度控制内循环
void FTC_FlyControl::Altitude_Inner_Loop(void)
{
	//PID
	if(!flag_height)
		pid[PIDVELZ].reset_I();
	int32_t tmp = pid[PIDVELZ].get_pid(this->AltSpeedError, 0.02f);
	this->AltThr = constrain_int32(tmp + 1000, 1400, 1800);	
}

void FTC_FlyControl::AltHoldReset(void)
{
	AltHold = nav.position.z;
}

/************************ (C) COPYRIGHT 2015 FTC *****END OF FILE**********************/
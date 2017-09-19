/******************** (C) COPYRIGHT 2015 FTC ***************************
 * ??		 :FTC
 * ???  :FTC_RC.cpp
 * ??    :????????
**********************************************************************************/
#include "FTC_RC.h"

FTC_RC rc;

FTC_RC::FTC_RC()
{
	Init();
}

void FTC_RC::Init(void)
{
	for (u8 i = 0; i < PITCH_LOOKUP_LENGTH; i++)
		lookupPitchRollRC[i] = (2500 + RC_EXPO * (i * i - 25)) * i * (int32_t) RC_RATE / 2500;
		
	for (u8 i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
			int16_t tmp = 10 * i - THR_MID;
			uint8_t y = 1;
			if (tmp > 0)
					y = 100 - THR_MID;
			if (tmp < 0)
					y = THR_MID;
			lookupThrottleRC[i] = 10 * THR_MID + tmp * (100 - THR_EXPO + (int32_t) THR_EXPO * (tmp * tmp) / (y * y)) / 10;
			lookupThrottleRC[i] = MINTHROTTLE + (int32_t) (MAXTHROTTLE - MINTHROTTLE) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
	}		
}

void FTC_RC::Cal_Command(void)
{
	  int32_t tmp, tmp2;
    for (u8 i = 0; i < 3; i++) 
		{	//??ROLL,PITCH,YAW???????
        tmp = min(abs(rawData[i] - RC_MID), 500);	
        if (i != 2) 
				{ // ROLL & PITCH
            tmp2 = tmp / 100;
						Command[i] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
        }
				else 
				{                // YAW
            Command[i] = -tmp;
        }
        if (rawData[i] < RC_MID)
            Command[i] = -Command[i];
    }

    tmp = constrain_uint16(rawData[THROTTLE], RC_MINCHECK, 2000);
    tmp = (uint32_t) (tmp - RC_MINCHECK) * 1000 / (2000 - RC_MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
		
		if(!ftc.f.ALTHOLD){
			Command[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
			Command[THROTTLE] = constrain_uint16(rawData[THROTTLE], RC_MINCHECK, 2000);
		}
		
		//-------------------????------------------
		if (abs(Command[YAW]) < 70 && rawData[THROTTLE] > RC_MINCHECK) 
		{
			int16_t dif = imu.angle.z - imu.magHold;
			if (dif <= -180)
				dif += 360;
			if (dif >= +180)
				dif -= 360;
			dif = -dif;
			
			Command[YAW] -= dif * fc.pid[PIDMAG].kP;  	
		} 	
		else
			imu.magHold = imu.angle.z;	
		
}


const uint8_t stick_min_flag[4] = {1<<0,1<<2,1<<4,1<<6}; 
const uint8_t stick_max_flag[4] = {1<<1,1<<3,1<<5,1<<7};
#define ROL_L 0x01
#define ROL_H 0x02
#define PIT_L	0x04
#define PIT_H 0x08
#define YAW_L 0x10
#define YAW_H 0x20
#define THR_L 0x40
#define THR_H 0x80

void FTC_RC::check_sticks(void)
{
	int i;
	static uint8_t rcDelayCommand;
	
	static uint8_t stick_flag = 0;

	for (i = 0; i < 4; i++) 
	{
			if(rawData[i]<900||rawData[i]>2000)	//????????????,?????
				break;
			
			if (rawData[i] < RC_MINCHECK)
					stick_flag |= stick_min_flag[i];  // check for MIN
			else
					stick_flag &= ~stick_min_flag[i];
			
			if (rawData[i] > RC_MAXCHECK)
					stick_flag |= stick_max_flag[i];  // check for MAX
			else
					stick_flag &= ~stick_max_flag[i]; 
	}	
	
	if(stick_flag&0xff)	//??????????????
	{
		if(rcDelayCommand < 250)
			rcDelayCommand++;
	}
	else
	{
		rcDelayCommand = 0;
		stick_flag &= 0;
	}	
		
	if (rcDelayCommand == 100) //2s: 20ms * 100
	{
		if (ftc.f.ARMED) //??????????
		{ 
			if((stick_flag & YAW_L)&&(stick_flag & THR_L))
			{
				ftc.f.ARMED = 0;	//??
			}
		}
		else
		{
			if((stick_flag & YAW_H)&&(stick_flag & THR_L))
			{
				if(ftc.f.CALIBRATED)
					ftc.f.ARMED = 1;	//??
				else
				{
					mpu6050.Gyro_CALIBRATED = 1;
				}
			}
		}
		stick_flag &= 0;
		rcDelayCommand = 0;
	}
}

void FTC_RC::CheckAUX(void)
{
	if(rc.rawData[AUX1]>0 && rc.rawData[AUX1]<2100)	//????1,???? ?? ????
	{

			if(rc.rawData[AUX1]>0 && rc.rawData[AUX1]<1300)
			{
				fc.flag_height = 0;
			}
			else if(rc.rawData[AUX1]>1400 && rc.rawData[AUX1]<1600)
			{
				
					fc.flag_height = 1;
				
			}
			else if(rc.rawData[AUX1]>1700 && rc.rawData[AUX1]<2100)
			{
				
					fc.flag_height = 2;
			}
	

	}
	if(rc.rawData[AUX2]>0 && rc.rawData[AUX2]<2100)	//????2,???? ????
	{
		if(rc.rawData[AUX2]>0 && rc.rawData[AUX2]<1300)
		{
			fc.flag_fly = 1;
			fc.flag_throw = 0;
			fc.flag_return = 0;
			//fc.flag_up_down = 0;
		}
		else if(rc.rawData[AUX2]>1400 && rc.rawData[AUX2]<1600)
		{
			if(fc.will_return)
					fc.flag_return = 1;
				else
				fc.flag_throw = 1;
		}
		else if(rc.rawData[AUX2]>1700 && rc.rawData[AUX2]<2100)
		{
			if(fc.will_return)
					fc.flag_return = 2;
				else
				fc.flag_throw = 2;
		}

	}
}

/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/
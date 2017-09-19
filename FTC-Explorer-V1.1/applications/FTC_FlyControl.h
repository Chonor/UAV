#ifndef __FTC_FLYCONTROL_H
#define __FTC_FLYCONTROL_H

#include "FTC_Config.h"

#define FLYANGLE_MAX 150  //??????17

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDANGLE,
    PIDMAG,
    PIDVELZ,
    PIDALT,
		PIDITEMS
};

class FTC_FlyControl
{

public:
	
	FTC_PID pid[PIDITEMS];
	Vector3f error[PIDITEMS];
	Vector3i setVelocity;
	uint8_t velocityControl;
	int32_t errorVelocityI;
  float record[PIDITEMS];

	u8 will_return;
	
	Vector3i velPIDTerm;

	int32_t AltSpeedError, AltHold, AltErr, autoUp, flag_throw, flag_fly, flag_height, Height, flag_return;
	float LastThr, NowThr, AltThr;
	FTC_FlyControl();

	void PID_Reset(void);
	void AltHoldReset(void);

	//??????
	void Attitude_Outter_Loop(void);

	//??????
	void Attitude_Inner_Loop(void);

	//??????
	void Altitude_Outter_Loop(void);

	//??????
	void Altitude_Inner_Loop(void);

private:
	
	uint8_t rollPitchRate;
	uint8_t yawRate;
	int32_t RateError[3];

	Vector3i velError;
	int16_t altHoldDeadband;

};

extern FTC_FlyControl fc;

#endif























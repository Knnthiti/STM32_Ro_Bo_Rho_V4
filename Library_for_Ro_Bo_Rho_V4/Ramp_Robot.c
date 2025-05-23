/*
 * Ramp_Robot.c
 *
 *  Created on: Nov 8, 2024
 *      Author: Knnn
 */
#include "Ramp_Robot.h"

////////////////////////////////////////////////////////////////////////
float _r     = 0.0f;
float _Rad_s = 0.0f;

void Cartesian_to_Polar(float Px, float Py){
	_r = (float)(sqrt((Px * Px) + (Py * Py)));
	_Rad_s = (float)(atan2(Py, Px));
}

float get_Polar_r(){
	return _r;
}

float get_Polar_Rad_s(){
	return _Rad_s;
}

////////////////////////////////////////////////////////////////////////
float _V_r     = 0.0f;
float _V_Rad_s = 0.0f;

float _Kp_r;
float _Ki_r;
float _Kd_r;
float _max_Vr;

void Setup_Ramp_Polar_r(float Kp_r ,float Ki_r ,float Kd_r ,float max_Vr){
	_Kp_r = Kp_r;
	_Ki_r = Ki_r;
	_Kd_r = Kd_r;
	_max_Vr = max_Vr;
}

float Proportiona_Vr = 0.0f;
float Integnator_Vr = 0.0f;
float Derivative_Vr = 0.0f;
float Past_Error_Vr = 0.0f;

void Ramp_Polar_r(float Diff_r){
	Proportiona_Vr = Diff_r;
	Integnator_Vr += Diff_r;
	Derivative_Vr = Diff_r - Past_Error_Vr;

	Past_Error_Vr = Diff_r;

	_V_r = (Proportiona_Vr * _Kp_r) + (Integnator_Vr * _Ki_r) + (Derivative_Vr * _Kd_r);

	if (_V_r > _max_Vr) {
		_V_r = _max_Vr;
	} else if (_V_r < -_max_Vr) {
		_V_r = -_max_Vr;
	}

	if ((_V_r < 0.05) && (_V_r > -0.05)) {
		_V_r = 0.0;
		Integnator_Vr = 0;
	}
}

float get_V_r(){
	return _V_r;
}

float get_V_Rad_s(){
	return _V_Rad_s;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
float __Vx = 0.0f;
float __Vy = 0.0f;
float __Vz = 0.0f;

void Polar_to_Cartesian(float V_r ,float _Rad_s){
	__Vx = (V_r * cos(_Rad_s));
	__Vy = (V_r * sin(_Rad_s));
}

float get_Cartesian_Vx(){
	return __Vx;
}

float get_Cartesian_Vy(){
	return __Vy;
}


float _Kp_Vz;
float _Ki_Vz;
float _Kd_Vz;
float _max_Vz;

void Setup_Ramp_Vz(float Kp_Vz ,float Ki_Vz ,float Kd_Vz,float max_Vz){
	_Kp_Vz = Kp_Vz;
	_Ki_Vz = Ki_Vz;
	_Kd_Vz = Kd_Vz;
	_max_Vz = max_Vz;
}

float Error_Vz = 0.0f;
float Proportiona_Vz = 0.0f;
float Integnator_Vz = 0.0f;
float Derivative_Vz = 0.0f;
float Past_Error_Vz = 0.0f;

float Ramp_Vz(float Set_point_Rad ,float head_ing){
	Error_Vz = Set_point_Rad - head_ing;
	Proportiona_Vz = Error_Vz;
	Integnator_Vz += Error_Vz;
	Derivative_Vz = Error_Vz - Past_Error_Vz;

	Past_Error_Vz = Error_Vz;

	__Vz = (Proportiona_Vz * _Kp_Vz) + (Integnator_Vz * _Ki_Vz) + (Derivative_Vz * _Kd_Vz);

	if (__Vz > _max_Vz) {
		__Vz = _max_Vz;
	} else if (__Vz < -_max_Vz) {
		__Vz = -_max_Vz;
	}

	if ((__Vz < 0.05) && (__Vz > -0.05)) {
	    __Vz = 0.0;
	    Integnator_Vz = 0;
	}
	return __Vz;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float _V_degree = 0.0f;
int16_t _DutyCycle = 0;

float _Kp_degree;
float _Ki_degree;
float _Kd_degree;
float _max_Count;
float _DutyCycle_MAX;

void Setup_Ramp_Count(float Kp_Count ,float Ki_Count ,float Kd_Count,float max_Count ,float DutyCycle_MAX){
	_Kp_degree = Kp_Count;
	_Ki_degree = Ki_Count;
	_Kd_degree = Kd_Count;
	_max_Count = max_Count;
	_DutyCycle_MAX = DutyCycle_MAX;
}

float Count_to_degree(int32_t Count){
	return (Count * (360.0f/_max_Count));
}

float Degree_to_Count(float Degree){
	return (Degree * (_max_Count/360.0f));
}

float Error_degree = 0.0f;
float Proportiona_degree = 0.0f;
float Integnator_degree = 0.0f;
float Derivative_degree = 0.0f;
float Past_Error_degree = 0.0f;

int16_t Ramp_Count(float Set_degree ,float degree){
	Error_degree = Set_degree - degree;
	Proportiona_degree = Error_degree;
	Integnator_degree += Error_degree;
	Derivative_degree = Error_degree - Past_Error_degree;

	Past_Error_degree = Error_degree;

	_V_degree = (Proportiona_degree * _Kp_degree) + (Integnator_degree * _Ki_degree) + (Derivative_degree * _Kd_degree);

	_DutyCycle = Degree_to_Count(_V_degree) * (_DutyCycle_MAX/_max_Count);
	return _DutyCycle;
}



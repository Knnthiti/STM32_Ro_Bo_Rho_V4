/*
 * Game_Play.c
 *
 *  Created on: Apr 25, 2025
 *      Author: Knnn
 */

#include "Game_Play.h"

uint8_t Past_move4 = 0;

long Time_GUN = 0;
float __Degree = 0.0f;
uint8_t Past_attack1 = 0;

uint8_t Past_attack2 = 0;

long Time_Juggling = 0;
uint8_t _Case_Juggling = 0;
uint8_t Past_attack3 = 0;

long Time_Reload = 0;
uint8_t _Case_Reload = 0;
uint8_t Past_attack4 = 0;

typedef struct{
	uint8_t status_GUN;
	uint8_t status_Reload;
	uint8_t status_Juggling;
}Robot;

Robot Status_Robot;

void Shoot_Ball(TIM_HandleTypeDef *PIN_PWM){
	if(Status_Robot.status_GUN == 1){
		digitalWrite("PA11" ,1);
	}else if((Status_Robot.status_GUN == 1) && ((uwTick - Time_GUN) >= 500)){
		digitalWrite("PA11" ,0);
		Motor_DutyCycle(PIN_PWM, Servo1, "PB00", 0);
		Status_Robot.status_GUN = 0;
	}

}

void Juggling(){
	switch (_Case_Juggling) {
	case 0:
		if ((Status_Robot.status_Juggling == 1) && ((uwTick - Time_Juggling) >= 500)) {
			digitalWrite("PA05", 1);
			_Case_Juggling = 1;
		}
		break;
	case 1:
		if ((Status_Robot.status_Juggling == 1) && ((uwTick - Time_Juggling) >= 1000)) {
			digitalWrite("PA05", 0);
			Motor_DutyCycle_EXTRA2(-1000);
			_Case_Juggling = 2;
		}
		break;
	case 2:
		if ((Status_Robot.status_Juggling == 1) && ((uwTick - Time_Juggling) >= 1500)) {
			Status_Robot.status_Juggling = 0;
			Motor_DutyCycle_EXTRA2(0);
			_Case_Juggling = 0;
		}
		break;
	}

}

void Reload(TIM_HandleTypeDef *PIN_PWM){
	switch (_Case_Reload) {
		case 0:
			if ((Status_Robot.status_Reload == 1) && ((uwTick - Time_Juggling) >= 500)) {
				digitalWrite("PA10", 1);
				_Case_Reload = 1;
			}
			break;
		case 1:
			if ((Status_Robot.status_Reload == 1) && ((uwTick - Time_Juggling) >= 1000)) {
				Status_Robot.status_Reload = 0;
				digitalWrite("PA10", 0);
				Motor_DutyCycle(PIN_PWM, Servo1, "PB00", 0);
				_Case_Reload = 0;
			}
			break;
		}
}

void Game_Play_ROBOT_1(TIM_HandleTypeDef *PIN_PWM ,TIM_HandleTypeDef *_Encoder){
	//Set up Juggling
	if((Str_PS2.moveBtnBit.move4 == 1) && (Past_move4 == 0)){
		Motor_DutyCycle_EXTRA2(500);
		digitalWrite("PA04", 1);
	}
	Past_move4 = Str_PS2.moveBtnBit.move4;
	//Set up Juggling

	//_Juggling
	if ((Str_PS2.attackBtnBit.attack3 == 1) && (Past_attack3 == 0)){
		Time_Juggling = uwTick;
		digitalWrite("PA04", 0);
		Status_Robot.status_Juggling = 1;
	}
	Juggling();
	Past_attack3 = Str_PS2.attackBtnBit.attack3;
	//_Juggling

	//_Reload
	if ((Str_PS2.attackBtnBit.attack4 == 1) && (Past_attack4 == 0)) {
		Time_Reload = uwTick;
		__Degree = 0.0f;
		Motor_DutyCycle(PIN_PWM, Servo1, "PB00", -1000);
		Status_Robot.status_Reload = 1;
	}
	Reload(PIN_PWM);
	Motor_DutyCycle_EXTRA1(Ramp_Count(__Degree, Count_to_degree(getCount(_Encoder, _EXTRA1))));
	Past_attack4 = Str_PS2.attackBtnBit.attack4;
	//_Reload

    //_charge_Ball
	if((Str_PS2.attackBtnBit.attack2 == 1) && (Past_attack2 == 0)){
		Motor_DutyCycle(PIN_PWM, Servo1, "PB00", 4000);
		__Degree = -135.0;
	}
	Past_attack2 = Str_PS2.attackBtnBit.attack2;
	//_charge_Ball

	//_Shoot_Ball
	if ((Str_PS2.attackBtnBit.attack1 == 1) && (Past_attack3 == 0)) {
		Time_GUN = uwTick;
		Status_Robot.status_GUN = 1;
	}
	Shoot_Ball(PIN_PWM);
	Past_attack1 = Str_PS2.attackBtnBit.attack1;
	//_Shoot_Ball
}

/*
 * Game_Play.h
 *
 *  Created on: Apr 25, 2025
 *      Author: Knnn
 */

#ifndef GAME_PLAY_H_
#define GAME_PLAY_H_

#include "main.h"
#include "Encoder_Motor.h"
#include "UART_ESP.h"
#include "Ramp_Robot.h"

void Shoot_Ball(TIM_HandleTypeDef *PIN_PWM);
void Juggling();
void Reload(TIM_HandleTypeDef *PIN_PWM);
void Game_Play_ROBOT_1();

#endif /* GAME_PLAY_H_ */

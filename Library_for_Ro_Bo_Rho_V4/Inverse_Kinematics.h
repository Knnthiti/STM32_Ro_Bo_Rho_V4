/*
 * Inverse_Kinematics.h
 *
 *  Created on: Nov 6, 2024
 *      Author: Knnn
 */

#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include "main.h"

#include "Ramp_Robot.h"

//https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
//inverse kinematic equations mecanum wheel

extern float Lx_;
extern float Ly_;
extern float Radius_wheel_;

void Setup_Inverse_Kinematic(float Lx ,float Ly ,float Radius_wheel);

extern float w_LF;
extern float w_LB;
extern float w_RF;
extern float w_RB;

void Inverse_Kinematic(float Vx ,float Vy ,float wz);
void Inverse_Kinematic_Lock_Direction(float Vx ,float Vy ,float wz ,float head_ing);

float get_w_LF();
float get_w_LB();
float get_w_RF();
float get_w_RB();

#endif /* INVERSE_KINEMATICS_H_ */

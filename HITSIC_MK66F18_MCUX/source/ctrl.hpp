/*
 * ctrl.hpp
 *
 *  Created on: 2020年11月21日
 *      Author: 刘馨元
 */

#ifndef CTRL_HPP_
#define CTRL_HPP_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sc_ftm.h"
#include "lib_pidctrl.h"
#include "sc_host.h"

#include "image.hpp"
#include "em.hpp"

extern float error;
extern float EM_RUN;
extern float IMAGE_RUN;
extern float servo_mid;
extern float motor_spdset;
extern float motorL_kp;
extern float motorL_ki;
extern float motorR_kp;
extern float motorR_ki;
extern float error_1;

extern int zebra_stop;
extern pidCtrl_t SERVO_PID;
extern pidCtrl_t MOTOR_PID;
extern float motor_spdL;
extern float motor_spdR;
extern float motor_pwmL;
extern float motor_pwmR;
extern float servo_pwm;
extern float motor_spdsetL;
extern float motor_spdsetR;
extern float K;
extern int imgmid;
extern int start_switch;

void CTRL_MOTOR(void);
void CTRL_SERVO(void);
void CTRL_Init(void);
#endif /* CTRL_HPP_ */

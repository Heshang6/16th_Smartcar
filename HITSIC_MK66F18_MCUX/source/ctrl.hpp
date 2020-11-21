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

#include "image.hpp"
#include "em.hpp"

extern float error;
extern float EM_RUN;
extern float IMAGE_RUN;
extern float servo_mid;
extern float motor_pwm;
extern pidCtrl_t SERVO_PID;
extern pidCtrl_t MOTOR_PID;

void CTRL_MOTOR(void);
void CTRL_SERVO(void);
void CTRL_Init(void);
#endif /* CTRL_HPP_ */

/*
 * ctrl.cpp
 *
 *  Created on: 2020年11月21日
 *      Author: 刘馨元
 */

#include "ctrl.hpp"


void CTRL_Init(void)
{
    pitMgr_t::insert(5U, 2U, CTRL_MOTOR, pitMgr_t::enable);
    pitMgr_t::insert(20U, 3U, CTRL_SERVO, pitMgr_t::enable);
    pitMgr_t::insert(20U, 4U, EM_GetError, pitMgr_t::enable);
}

extern float em_error;
extern float AD[8];

float error = 0;
float servo_pwm = 0.0f;

float EM_RUN = 0;
float IMAGE_RUN = 1;
float servo_mid = 7.3f;
float motor_pwm = 20.0f;
/** 舵机PID结构体 */
pidCtrl_t SERVO_PID =
{
    .kp = 0.033f, .ki = 0.0f, .kd = 0.044f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};
/** 电机PID结构体 */
pidCtrl_t MOTOR_PID =
{
    .kp = 0.30f, .ki = 0.20f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};

////////////////////////////////////////////
//功能：舵机控制
//输入：
//输出：
//备注：
///////////////////////////////////////////
void CTRL_SERVO()
{
    if((IMAGE_RUN == 1)&&(EM_RUN == 0))
    {
        error = 94 - int(mid_line[prospect]);
        servo_pwm = servo_mid + PIDCTRL_UpdateAndCalcPID(&SERVO_PID, error);
        if(servo_pwm<6.8)
            servo_pwm=6.8;
        else if(servo_pwm>8.3)
            servo_pwm=8.3;
        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_pwm);
    }
    if((IMAGE_RUN == 0)&&(EM_RUN == 1))
    {
        error = em_error;
        servo_pwm = servo_mid + PIDCTRL_UpdateAndCalcPID(&SERVO_PID, error);
        if(servo_pwm<6.8)
            servo_pwm=6.8;
        else if(servo_pwm>8.3)
            servo_pwm=8.3;
        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_pwm);
    }
}

////////////////////////////////////////////
//功能：电机控制
//输入：
//输出：
//备注：
///////////////////////////////////////////
void CTRL_MOTOR()
{
//    if((img_protect == 0)&&(zebra_change != 2));
    if(img_protect == 0)
    {
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000U,motor_pwm);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000U,0);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000U,0);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000U,motor_pwm);
    }
    if(img_protect == 1)
    {
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000U,0);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000U,0);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000U,0);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000U,0);
    }
}


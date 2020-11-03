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
   // pitMgr_t::insert(20U, 4U, EM_GetError, pitMgr_t::enable);
}

extern float em_error;
extern float AD[8];

float errorl_0 = 0;
float errorl_1 = 0;
float errorr_0 = 0;
float errorr_1 = 0;//电机左右轮两次误差
float error_0 = 0.0f;//舵机误差
float error_1 = 0.0f;
float servo_pwm = 0.0f;
float motor_spdL = 0.0f;//左右轮实际速度
float motor_spdR = 0.0f;
float motor_pwmL = 0.0f;//左右轮输出
float motor_pwmR = 0.0f;
float motor_spdsetL = 0.0f;//左右轮期望速度
float motor_spdsetR = 0.0f;

int zebra_stop = 0;
int start_switch = 0;
int imgmid = 94;
float K = 1;
float motor_spdset = 2; //实际期望速度
float EM_RUN = 0;
float IMAGE_RUN = 1;
float servo_mid = 7.5f;
float motorL_kp = 0.03f;
float motorL_ki = 0.03f;
float motorR_kp = 0.03f;
float motorR_ki = 0.03f;

/** 舵机PID结构体 */
pidCtrl_t SERVO_PID =
{
    .kp = 0.033f, .ki = 0.0f, .kd = 0.044f,
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
    if(start_switch == 0)
    {
    if((IMAGE_RUN == 1)&&(EM_RUN == 0))
    {
        error_1 = imgmid - int(mid_line[prospect]);
//        if(error_1-error_0>20)
//        {
//            error_1 = error_0;
//        }
        servo_pwm = servo_mid + SERVO_PID.kp*error_1 + SERVO_PID.kd*(error_1-error_0);
        if(servo_pwm<6.8)
            servo_pwm=6.8;
        else if(servo_pwm>8.2)
            servo_pwm=8.2;
        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_pwm);
        error_0 = error_1;
    }
    if((IMAGE_RUN == 0)&&(EM_RUN == 1))
    {
        error_1 = em_error;
        servo_pwm = servo_mid + PIDCTRL_UpdateAndCalcPID(&SERVO_PID, error_1);
        if(servo_pwm<6.8)
            servo_pwm=6.8;
        else if(servo_pwm>8.2)
            servo_pwm=8.2;
        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_pwm);
    }
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
     if(start_switch == 0)
     {
              motor_spdR = -SCFTM_GetSpeed(FTM1)*0.034778;
              SCFTM_ClearSpeed(FTM1);
              motor_spdL = SCFTM_GetSpeed(FTM2)*0.034778;
              SCFTM_ClearSpeed(FTM2);
              if(zebra_stop == 1)
              {
                  motor_spdset = 0;
              }
              if(servo_pwm > servo_mid)//左转
              {
                  motor_spdsetR = motor_spdset;
                  motor_spdsetL = motor_spdset*K*(-0.2229*servo_pwm*servo_pwm*servo_pwm+5.2118*servo_pwm*servo_pwm-41.092*servo_pwm+110.08);
              }
              if(servo_pwm < servo_mid)//右转
              {
                  motor_spdsetL = motor_spdset;
                  motor_spdsetR = motor_spdset*K/(-0.2229*servo_pwm*servo_pwm*servo_pwm+5.2118*servo_pwm*servo_pwm-41.092*servo_pwm+110.08);
              }
              if(servo_pwm == servo_mid)
              {
                  motor_spdsetL = motor_spdset;
                  motor_spdsetR = motor_spdset;
              }

              errorl_1 = motor_spdsetL-motor_spdL;
              errorr_1 = motor_spdsetR-motor_spdR;
              motor_pwmL += motorL_kp *(errorl_1 - errorl_0) + motorL_ki*(errorl_1);
              motor_pwmR += motorR_kp *(errorr_1 - errorr_0) + motorR_ki*(errorr_1);
              errorl_0 = errorl_1;
              errorr_0 = errorr_1;
              if(motor_pwmL > 99)
                  motor_pwmL = 99;
              if(motor_pwmL < -99)
                  motor_pwmL = -99;
              if(motor_pwmR > 99)
                  motor_pwmR = 99;
              if(motor_pwmR < -99)
                  motor_pwmR = -99;
              if(img_protect == 1)
              {
                  SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000U,0);
                  SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000U,0);
                  SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000U,0);
                  SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000U,0);
              }
              else
              {
                     if(motor_pwmL>0)
                     {
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000U,motor_pwmL);
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000U,0);
                     }
                     else
                     {
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000U,0);
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000U,-motor_pwmL);
                     }
                     if(motor_pwmR>0)
                     {
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000U,0);
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000U,motor_pwmR);
                     }
                     else
                     {
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000U,-motor_pwmR);
                         SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000U,0);
                     }
              }
     }
}


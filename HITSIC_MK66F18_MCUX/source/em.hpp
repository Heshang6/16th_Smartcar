/*
 * em.hpp
 *
 *  Created on: 2020年11月20日
 *      Author: 刘馨元
 */

#ifndef EM_HPP_
#define EM_HPP_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "sc_adc.h"
#include "drv_disp_ssd1306_port.hpp"


#define SampleTimes 25
#define ChannelTimes 8

extern float AD[8];
extern float em_error;

void LV_Sample(void);
void LV_Get_Val(void);
void swap(uint32_t * a, uint32_t * b);
void EM_GetError(void);



#endif /* EM_HPP_ */

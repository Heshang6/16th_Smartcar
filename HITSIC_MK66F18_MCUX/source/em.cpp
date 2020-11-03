/*
 * em.cpp
 *
 *  Created on: 2020年11月20日
 *      Author: 刘馨元
 */

#include "em.hpp"

uint32_t LV_Temp[8][25];
const uint32_t channels[8] = {16, 23, 17, 18, 10, 11, 12, 13};
int LV[8];
int AD[8];
float em_error;

void LV_Sample(void)                             // ad采集函数
{
    for (uint8_t j = 0; j < ChannelTimes; j++)
    {
        for(uint8_t i=0;i<SampleTimes;i++)
        {
            LV_Temp[j][i]=SCADC_Sample(ADC0,0,channels[j]);//左边电感
        }
    }
}

void LV_Get_Val(void)//约0.3mS                  //对采集的值滤波
{
    for(uint8_t i=0;i<ChannelTimes;i++)
    {
        for(uint8_t j=0;j<SampleTimes;j++)
        {
            if(LV_Temp[i][j]>500)//剔除毛刺信号
            {
                LV_Temp[i][j]=500;
            }
        }
    }

    //排序
    for(uint8_t k=0;k<ChannelTimes;k++)
    {
        for(uint8_t i=0;i<SampleTimes-1;i++)
        {
            for(uint8_t j=i+1;j<SampleTimes;j++)
            {
                if(LV_Temp[k][i]>LV_Temp[k][j])
                    swap(&LV_Temp[k][i],&LV_Temp[k][j]);//交换，swap函数自己写
            }
        }
    }

    for(uint8_t k=0;k<ChannelTimes;k++)
    {
        LV[k]=0;
        uint32_t MinLVGot = LV_Temp[k][0];
        for(uint8_t i=3;i<SampleTimes-3;i++)
        {
            LV[k]+=(float)LV_Temp[k][i];
        }
        LV[k]=LV[k]/(SampleTimes-6);
        /*if( LV[k] < MinLVGot )
        {
            LV[k] = MinLVGot;
        }*/
    }

    AD[0] = LV[0];
    AD[1] = LV[1];
    AD[2] = LV[2];
    AD[3] = LV[3];
    AD[4] = LV[4];
    AD[5] = LV[5];
    AD[6] = LV[6];
    AD[7] = LV[7];
    AD[8] = LV[8];//注意这里不要直接用LV数组
}

void swap(uint32_t * a, uint32_t * b)
{
    uint32_t t = 0;
    t = *a;
    *a = *b;
    *b = t;
}

void EM_GetError(void)
{
    LV_Sample();
    LV_Get_Val();
    em_error = (float)((AD[6]-AD[0])/(AD[6]*AD[0]));
}


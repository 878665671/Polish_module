/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-07     yy       the first version
 */
#ifndef APPLICATIONS_APP_DATA_CLASS_H_
#define APPLICATIONS_APP_DATA_CLASS_H_
#include <board.h>
/* 线程 */


/* **************** */

/* adc */
#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */

#define Pressure_sensor1_channel  10    /* 压力传感器 1*/
#define Pressure_sensor2_channel  11    /* 压力传感器 2*/
#define Pressure_sensor3_channel  12    /* 压力传感器 3*/

#define Temp_sensor1_channel  1    /* 温度传感器1*/
#define Temp_sensor2_channel  2    /* 温度传感器2*/

#define Grinding_Motor1_channel 3 /* 1号打磨电机通道   ch4*/

#define Grinding_Motor2_channel 5 /* 2号打磨电机通道 ch7*/
#define Vacuuming_Motor_channel 0 /* 吸尘电机通道*/
#define Dust_Motor_channel      6 /* 扫灰电机通道*/

//#define REFER_WEIGHT       50         /* 参考极限压力，单位gk */
/* ************************************* */






extern rt_uint16_t kpa_value;//压力传感器原始值
extern float temp;             //温度
extern  rt_uint16_t speed;            //转速

extern float Grinding1; //打磨1电机电流
extern float Grinding2; //打磨2电机电流
extern float Vacuuming; //吸尘电机电流
extern float Dust;//扫灰电机电流
int PIN_FLAG(void);
#endif /* APPLICATIONS_APP_DATA_CLASS_H_ */


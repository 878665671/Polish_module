/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-07     yy       the first version
 */
#ifndef APPLICATIONS_APP_CONTROL_CLASS_H_
#define APPLICATIONS_APP_CONTROL_CLASS_H_


/* 打磨电机 */
#define Grinding_PIN GET_PIN(B,12)      /* 打磨电机1继电器引脚  */
#define Grinding_PIN_INIT PIN_LOW       /* 打磨电机1继电器引脚初始电平  */
#define Grinding_PIN_ENABLE PIN_HIGH    /* 打磨电机1启动  */
#define Grinding_PIN_DISABLE PIN_LOW    /* 打磨电机1停止  */

#define Grinding_PIN2 GET_PIN(D,11)      /* 打磨电机2继电器引脚  */
#define Grinding_PIN2_INIT PIN_LOW       /* 打磨电机2继电器引脚初始电平  */
#define Grinding_PIN2_ENABLE PIN_HIGH    /* 打磨电机2启动  */
#define Grinding_PIN2_DISABLE PIN_LOW    /* 打磨电机2停止  */
//-------------------------------
#define Grinding_FLAG_PIN GET_PIN(B,13) /* 打磨电机1继电器吸合反馈引脚  */
#define Grinding_FLAG_FLAG PIN_HIGH      /* 打磨电机1继电器吸合反馈引脚电平  */
#define Grinding_FLAG2_PIN GET_PIN(D,12) /* 打磨电机2继电器吸合反馈引脚  */
#define Grinding_FLAG2_FLAG PIN_HIGH      /* 打磨电机2继电器吸合反馈引脚电平  */
/* *************** */

/* 吸尘电机 */
#define Vacuuming_PIN GET_PIN(D,9)   /* 吸尘电机继电器引脚  */
#define Vacuuming_PIN_INIT PIN_LOW      /* 吸尘电机继电器引脚初始电平  */
#define Vacuuming_PIN_ENABLE PIN_HIGH    /* 吸尘电机启动  */
#define Vacuuming_PIN_DISABLE PIN_LOW    /* 吸尘电机停止  */
//-------------------------------
#define Vacuuming_FLAG_PIN GET_PIN(D,10) /* 吸尘电机继电器吸合反馈引脚  */
#define Vacuuming_FLAG_FLAG PIN_HIGH      /* 吸尘电机继电器吸合反馈引脚电平  */
/* *************** */


/* 扫灰电机 */
#define Dust_PIN GET_PIN(E,14)   /* 扫灰电机继电器引脚  */
#define Dust_PIN_INIT PIN_LOW      /* 扫灰电机继电器引脚初始电平  */
#define Dust_PIN_ENABLE PIN_HIGH    /* 扫灰电机启动  */
#define Dust_PIN_DISABLE PIN_LOW    /* 扫灰电机停止  */
//-------------------------------
#define Dust_FLAG_PIN GET_PIN(E,15) /* 扫灰电机继电器吸合反馈引脚  */
#define Dust_FLAG_FLAG PIN_HIGH      /* 扫灰电机继电器吸合反馈引脚电平  */
/* *************** */

/* 下压电机 */
#define DIR_PIN GET_PIN(C,9)       /* 下压电机方向引脚  */
#define DIR_PIN_INIT PIN_LOW      /* 下压电机方向引脚初始电平  */
#define DIR_PIN_UP PIN_LOW        /* 上升 */
#define DIR_PIN_DOWN PIN_HIGH      /* 下降  */
#define PWM_DEV_NAME        "pwm3"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL     3       /* PWM通道  PB0 */

extern struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */

/* *************** */

/* 下压上限位 */
#define KEY_PIN_NUM GET_PIN(B,8)   /* 限位开关引脚  */
#define KEY_PIN_FALG PIN_HIGH
/* ************************************* */

rt_uint8_t Grinding_Motor1_State(void);
rt_uint8_t Grinding_Motor2_State(void);
rt_uint8_t Vacuuming_Motor_State(void);
rt_uint8_t Dust_Motor_State(void);
rt_uint8_t Down_Motor_State(void);


void Grinding_ON_OFF(rt_uint8_t isEnable);
void Vacuuming_ON_OFF(rt_uint8_t isEnable);
void Dust_ON_OFF(rt_uint8_t isEnable);
void Down_pressure_motor(rt_uint8_t isValue);
void Down_pressure_motor_State(rt_uint8_t state);
void Down_pressure_init_flag(void);
void Mechanical_init(void);
//void Down_pressure_motor_speed_dir(rt_uint32_t isValue);
#endif /* APPLICATIONS_APP_CONTROL_CLASS_H_ */



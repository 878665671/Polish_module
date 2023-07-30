/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-07     yy       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "app_control_class.h"

struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */
static rt_uint8_t Down_State = 0xA0;
static rt_uint8_t State;

int PIN_FLAG(void)
{
    return rt_pin_read(KEY_PIN_NUM);
}
///* 下压上限位中断服务函数 */
//void beep_on(void *args)
//{
//    Down_pressure_motor(0);
//    rt_kprintf("turn on beep!\n");
//
//}

static int control_class_init(void)
{
    rt_uint32_t period;

    period = 33333;    /* 30k */

    /* 设置打磨电机引脚脚模式为输出 */
    rt_pin_mode(Grinding_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(Grinding_PIN2, PIN_MODE_OUTPUT);
    /* 设置打磨电机引脚默认电平 */
    rt_pin_write(Grinding_PIN, Grinding_PIN_INIT);
    rt_pin_write(Grinding_PIN2, Grinding_PIN2_INIT);
    /* 设置打磨电机继电器反馈引脚脚模式为输入 */
    rt_pin_mode(Grinding_FLAG_PIN, PIN_MODE_INPUT);
    rt_pin_mode(Grinding_FLAG2_PIN, PIN_MODE_INPUT);

    /* 设置吸尘电机引脚脚模式为输出 */
    rt_pin_mode(Vacuuming_PIN, PIN_MODE_OUTPUT);
    /* 设置吸尘电机引脚默认电平 */
    rt_pin_write(Vacuuming_PIN, Vacuuming_PIN_INIT);
    /* 设置吸尘电机继电器反馈引脚脚模式为输入 */
    rt_pin_mode(Vacuuming_FLAG_PIN, PIN_MODE_INPUT);

    /* 设置扫灰电机引脚脚模式为输出 */
    rt_pin_mode(Dust_PIN, PIN_MODE_OUTPUT);
    /* 设置扫灰电机引脚默认电平 */
    rt_pin_write(Dust_PIN, Dust_PIN_INIT);
    /* 设置扫灰电机继电器反馈引脚脚模式为输入 */
    rt_pin_mode(Dust_FLAG_PIN, PIN_MODE_INPUT);

    /* 设置DIR引脚脚模式为输出 */
    rt_pin_mode(DIR_PIN, PIN_MODE_OUTPUT_OD);
    /* 设置DIR引脚 默认电平*/
    rt_pin_write(DIR_PIN, DIR_PIN_INIT);




    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
       rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
       return RT_ERROR;
    }

    /* 设置PWM周期和脉冲宽度默认值 */
    if(RT_EOK!= rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, period / 2))
//        rt_pwm_set_pulse(pwm_dev, PWM_DEV_CHANNEL, 100000);
        rt_kprintf("pwm_set failed\n");
//    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
//    if(RT_EOK != rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL))
//        rt_kprintf("pwm_enable failed\n");

    /* 限位引脚为输入模式 */

    rt_pin_mode(KEY_PIN_NUM, PIN_MODE_INPUT_PULLUP);
//    /* 绑定中断，下降沿模式，回调函数名为beep_on */
//    ret = rt_pin_attach_irq(KEY_PIN_NUM, 3, beep_on, RT_NULL);
//    if (ret == RT_EOK)
////        rt_kprintf("rt_pin_attach_irq success!\n" );
//
//    /* 使能中断 */
//    ret = rt_pin_irq_enable(KEY_PIN_NUM, PIN_IRQ_ENABLE);
//    if (ret == RT_EOK)
////        rt_kprintf("rt_pin_irq_enable success!\n");

    return RT_EOK;
}


/*************************************************************
** Function name:       Grinding_Motor1_State
** Descriptions:        打磨电机1状态
** Input parameters:    None
** Output parameters:   None
** Returned value:      return：A0停止 ，A1运行
** Remarks:             None
*************************************************************/
rt_uint8_t Grinding_Motor1_State(void)
{
    if(Grinding_FLAG_FLAG == rt_pin_read(Grinding_FLAG_PIN))
    {
//        rt_kprintf("Grinding motor1 RUN!\n");
        return 0xA1;
    }

    else {
//        rt_kprintf("Grinding motor1 Stop!\n");
        return 0xA0;
    }
}


/*************************************************************
** Function name:       Grinding_Motor2_State
** Descriptions:        打磨电机2状态
** Input parameters:    None
** Output parameters:   None
** Returned value:      return：A0停止 ，A1运行
** Remarks:             None
*************************************************************/
rt_uint8_t Grinding_Motor2_State(void)
{
    if(Grinding_FLAG2_FLAG == rt_pin_read(Grinding_FLAG2_PIN))
    {
//        rt_kprintf("Grinding motor2 RUN!\n");
        return 0xA1;
    }

    else {
//        rt_kprintf("Grinding motor2 Stop!\n");
        return 0xA0;
    }
}


/*************************************************************
** Function name:       Vacuuming_Motor_State
** Descriptions:        吸尘电机状态
** Input parameters:    None
** Output parameters:   None
** Returned value:      return：A0停止 ，A1运行
** Remarks:             None
*************************************************************/
rt_uint8_t Vacuuming_Motor_State(void)
{
    if(Vacuuming_FLAG_FLAG == rt_pin_read(Vacuuming_FLAG_PIN))
    {
//        rt_kprintf("Vacuuming motor RUN!\n");
        return 0xA1;
    }

    else {
//        rt_kprintf("Vacuuming motor Stop!\n");
        return 0xA0;
    }
}


/*************************************************************
** Function name:       Dust_Motor_State
** Descriptions:        扫灰电机状态
** Input parameters:    None
** Output parameters:   None
** Returned value:      return：A0停止 ，A1运行
** Remarks:             None
*************************************************************/
rt_uint8_t Dust_Motor_State(void)
{
    if(Dust_FLAG_FLAG == rt_pin_read(Dust_FLAG_PIN))
    {
//        rt_kprintf("Dust motor RUN!\n");
        return 0xA1;
    }

    else {
//        rt_kprintf("Dust motor Stop!\n");
        return 0xA0;
    }
}

/*************************************************************
** Function name:       Down_Motor_State
** Descriptions:        下压电机状态
** Input parameters:    None
** Output parameters:   None
** Returned value:      return：A0停止 ，A1上升，A2下降
** Remarks:             None
*************************************************************/
rt_uint8_t Down_Motor_State(void)
{

    if(Down_State == 0xA1)
    {
//        rt_kprintf("Down Motor UP!\n");/
    }

    else if(Down_State == 0xA2)
    {
//        rt_kprintf("Down Motor Dow!\n");
    }

    else if(Down_State == 0xA0)
    {
//        rt_kprintf("Down Motor Stop!\n");
    }

    return Down_State;
}

/*************************************************************
** Function name:       Grinding_ON_OFF
** Descriptions:        打磨启停
** Input parameters:    isEnable：1：启动能 0：停止
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Grinding_ON_OFF(rt_uint8_t isEnable)
{
    if(isEnable == 1)
    {
        rt_pin_write(Grinding_PIN, Grinding_PIN_ENABLE);
        rt_pin_write(Grinding_PIN2, Grinding_PIN2_ENABLE);
        rt_thread_mdelay(5);
        Grinding_Motor1_State();
    }

    else if(isEnable == 0){
        rt_pin_write(Grinding_PIN, Grinding_PIN_DISABLE);
        rt_pin_write(Grinding_PIN2, Grinding_PIN2_DISABLE);
        rt_thread_mdelay(5);
        Grinding_Motor1_State();
    }
}

/*************************************************************
** Function name:       Vacuuming_ON_OFF
** Descriptions:        吸尘启停
** Input parameters:    isEnable：1：启动能 0：停止
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Vacuuming_ON_OFF(rt_uint8_t isEnable)
{
    if(isEnable == 1)
    {
        rt_pin_write(Vacuuming_PIN, Vacuuming_PIN_ENABLE);
        rt_thread_mdelay(5);
        Vacuuming_Motor_State();
    }
    else if(isEnable == 0){
        rt_pin_write(Vacuuming_PIN, Vacuuming_PIN_DISABLE);
        rt_thread_mdelay(5);
        Vacuuming_Motor_State();
    }
}

/*************************************************************
** Function name:       Dust_ON_OFF
** Descriptions:        扫灰启停
** Input parameters:    isEnable：1：启动能 0：停止
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Dust_ON_OFF(rt_uint8_t isEnable)
{
    if(isEnable == 1)
    {
        rt_pin_write(Dust_PIN, Dust_PIN_ENABLE);
        rt_thread_mdelay(5);
        Dust_Motor_State();
    }
    else if(isEnable == 0){
        rt_pin_write(Dust_PIN, Dust_PIN_DISABLE);
        rt_thread_mdelay(5);
        Dust_Motor_State();
    }
}



/*************************************************************
** Function name:       Down_pressure_motor
** Descriptions:        下压电机控制
** Input parameters:    isValue：0停止 ，1上升 ，2下降
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Down_pressure_motor(rt_uint8_t isValue)
{
    rt_err_t ret;

    if(isValue == 1 && State != 1)
    {
//        if(KEY_PIN_FALG != PIN_FLAG())
//        {
            rt_pin_mode(DIR_PIN, PIN_MODE_INPUT);
            rt_pin_mode(DIR_PIN, PIN_MODE_OUTPUT);
            rt_pin_write(DIR_PIN, DIR_PIN_UP);
            ret = rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
//        }

        if(ret == RT_EOK)
        {
            Down_State = 0xA1;
        }
    }
    else if(isValue == 0){
        ret = rt_pwm_disable(pwm_dev, PWM_DEV_CHANNEL);
        if(ret == RT_EOK)
        {
            Down_State = 0xA0;
        }
    }
    else if(isValue == 2 && State != 2)
    {
        rt_pin_mode(DIR_PIN, PIN_MODE_INPUT);
        rt_pin_mode(DIR_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(DIR_PIN, DIR_PIN_DOWN);

        ret = rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
        if(ret == RT_EOK)
        {
            Down_State = 0xA2;
        }
    }
}


/*************************************************************
** Function name:       Down_pressure_motor_State
** Descriptions:        下压电机状态
** Input parameters:    isValue：0停止 ，1上升 ，2下降
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Down_pressure_motor_State(rt_uint8_t state)
{
    State = state;
}
/*************************************************************
** Function name:       Down_pressure_init_flag
** Descriptions:        下压电机初始化上升到位标志
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Down_pressure_init_flag(void)
{
    Down_pressure_motor(1); /* 下压电机上升 */
    while(KEY_PIN_FALG != PIN_FLAG())
        {
//            rt_thread_mdelay(1);
            rt_kprintf("Wait for motor reset!\n");
        }
    Down_pressure_motor(0); /* 下压电机停止 */

}

/*************************************************************
** Function name:       Mechanical_init
** Descriptions:        机械初始化
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Mechanical_init(void)
{
    Grinding_ON_OFF(0);     /* 关闭打磨电机继电器 */
    Vacuuming_ON_OFF(0);    /* 关闭吸尘电机继电器 */
    Dust_ON_OFF(0);         /* 关闭扫灰尘电机继电器 */
    Down_pressure_init_flag();/* 等待下压电机上升到位 */
}

INIT_APP_EXPORT(control_class_init);




static void motorcmd(int argc, char**argv)
{
    if (argc < 3)
    {
        rt_kprintf("Please input'motorcmd <motor type> <on|off>'\n");
        return;
    }
    if (!rt_strcmp(argv[1], "dust"))
    {
        if (!rt_strcmp(argv[2], "on"))
            Dust_ON_OFF(1);
        else if(!rt_strcmp(argv[2], "off"))
            Dust_ON_OFF(0);
    }
    else if(!rt_strcmp(argv[1], "vacuuming"))
    {
        if (!rt_strcmp(argv[2], "on"))
            Vacuuming_ON_OFF(1);
        else if(!rt_strcmp(argv[2], "off"))
            Vacuuming_ON_OFF(0);
    }
    else if(!rt_strcmp(argv[1], "grinding"))
    {
        if (!rt_strcmp(argv[2], "on"))
            Grinding_ON_OFF(1);
        else if(!rt_strcmp(argv[2], "off"))
            Grinding_ON_OFF(0);
    }
    else {
        rt_kprintf("Please input'motorcmd <motor type> <on|off|up|dow|stop>'\n");
    }
}


MSH_CMD_EXPORT(motorcmd, motorcmd smple <motor type> <on|off>);

void  Down_Motor_UP(void)
{
    Down_pressure_motor(1);
}


void  Down_Motor_Dow(void)
{
    Down_pressure_motor(2);

}
void  Down_Motor_Stop(void)
{
    Down_pressure_motor(0);

}


MSH_CMD_EXPORT(Down_Motor_UP,Down_Motor_UP);

MSH_CMD_EXPORT(Down_Motor_Dow,Down_Motor_Dow);
MSH_CMD_EXPORT(Down_Motor_Stop,Down_Motor_Stop);

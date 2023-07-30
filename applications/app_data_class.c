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
#include "app_data_class.h"
#include "MLX90614.h"

#include <rtdbg.h>

#define VALUE 0.7890  //电机电流采样芯片校准
#define Millivolts_per_amp  0.066  //(意瑞) CH70120CB3PR
#define NUM 5   //滤波次数
#define VDD 3.3 //参考电压

#define THREAD_PRIORITY      22
#define THREAD_STACK_SIZE    1024 + 512
#define THREAD_TIMESLICE     5

#define ENCODER_RESOLUTION  4   // 光电编码器分辨率，每转500个脉冲
#define MEASURE_PERIOD_MS   100   // 测量周期，100ms

static rt_uint32_t pulse_count = 0; // 记录脉冲数
static rt_uint32_t last_pulse_count = 0; // 上一次的脉冲数
static rt_uint32_t last_measure_time = 0; // 上一次测量的时间


rt_uint16_t kpa_value;//压力传感器原始值
 float temp;             //温度
 rt_uint16_t speed;            //转速

 float Grinding1; //打磨1电机电流
 float Grinding2; //打磨2电机电流
 float Vacuuming; //吸尘电机电流
 float Dust;//扫灰电机电流

/* 光电编码器 */
#define PULSE_ENCODER_DEV_NAME    "pulse1"    /* 脉冲编码器名称 */
static rt_device_t pulse_encoder_dev;                /* 脉冲编码器设备句柄 */
/* ************************************** */

/* 压力传感器 */
static rt_adc_device_t adc_dev;                /* ADC设备句柄   */
/* ************************************* */


static void calculate_speed()
{
    rt_uint32_t current_time;
    rt_uint32_t delta_pulse_count;
    rt_uint32_t delta_time;
    float temp_;

    rt_thread_mdelay(MEASURE_PERIOD_MS); // 等待测量周期
    rt_device_read(pulse_encoder_dev, 0, &pulse_count, 1);
    current_time = rt_tick_get(); // 获取当前时间
    delta_pulse_count = pulse_count - last_pulse_count; // 计算脉冲数变化
    delta_time = current_time - last_measure_time; // 计算时间差
//    rt_kprintf("delta_pulse_count = %d,delta_time = %d\r\n",delta_pulse_count,delta_time);
    temp_ = delta_pulse_count * 60.0 / (ENCODER_RESOLUTION * delta_time / RT_TICK_PER_SECOND); // 计算转速
//    rt_kprintf("temp_: %f RPM\n", temp_);
    speed = (rt_uint32_t)temp_;
//    rt_kprintf("Speed: %d RPM\n", (rt_uint32_t)speed); // 打印转速
//    rt_kprintf("pulse_count: %d \n", pulse_count); // 打印转速
    last_pulse_count = pulse_count; // 更新脉冲数
    last_measure_time = current_time; // 更新时间
    rt_thread_mdelay(MEASURE_PERIOD_MS); // 等待测量周期
}



// 线程1的入口函数
static void thread1_entry(void *parameter)
{
    rt_uint32_t kpa1_temp,kpa2_temp,kpa3_temp; /*3个压力传感器原始值临时变量*/

    float temp3;        /*2个温度传感器临时变量*/
    rt_uint32_t temp1,temp2; //(差分计算)
    while (1)
    {
        /*读取采样值*/
                                                    /*压力读取*/
        for (int var = 0; var < NUM; var++) {        /*均值滤波*/

//            kpa1_temp = rt_adc_read(adc_dev, Pressure_sensor1_channel); /* 备用 */
            kpa2_temp += rt_adc_read(adc_dev, Pressure_sensor2_channel);
            kpa3_temp += rt_adc_read(adc_dev, Pressure_sensor3_channel);
        }


//        kpa1_temp = kpa1_temp / NUM;
        kpa2_temp = kpa2_temp / NUM;
        kpa3_temp = kpa3_temp / NUM;
///
//         rt_kprintf("kpa2_temp =%d,kpa3_temp =%d\n",kpa2_temp,kpa3_temp);
        rt_kprintf("%d,%d\n",kpa2_temp,kpa3_temp);

//        rt_kprintf("%d,%d\n",rt_adc_read(adc_dev, Pressure_sensor2_channel),rt_adc_read(adc_dev, Pressure_sensor3_channel));
//        rt_kprintf("kpa2_temp =%d\n",kpa2_temp);

        kpa_value = (kpa2_temp + kpa3_temp) / 2;

        rt_kprintf("kpa_value = %d\r\n",kpa_value);
                                                   /*温度读取*/
        for (int var = 0; var < NUM; var++)
        {                                           /*均值滤波*/
//            temp1 = rt_adc_read(adc_dev, Temp_sensor1_channel);
//            temp2 = rt_adc_read(adc_dev, Temp_sensor2_channel); /* TP100备用 */
            temp3 += SMBus_ReadTemp();
        }
//        temp1 = temp1 / NUM;
//        temp2 = temp2 / NUM;
        temp3 = temp3 / NUM;
        temp = temp3;

//        rt_kprintf("temp1 = %f,temp2 = %f\n",temp1,temp2);
//        temp1 = temp1 - temp2;
//        rt_kprintf("temp3 = %f\n",temp3);

                                                     /* 计算转速 */
        calculate_speed();
                                                    /* 计算电机电流 */
        for (int var = 0; var < NUM; var++)
        {                                           /*均值滤波*/

            Grinding1 += ((rt_adc_read(adc_dev, Grinding_Motor1_channel) * VDD / 4095) - 0.5 * VDD) / Millivolts_per_amp * VALUE;
            Grinding2 += ((rt_adc_read(adc_dev, Grinding_Motor2_channel) * VDD / 4095) - 0.5 * VDD) / Millivolts_per_amp * VALUE ;
            Vacuuming += ((rt_adc_read(adc_dev, Vacuuming_Motor_channel) * VDD / 4095) - 0.5 * VDD) / Millivolts_per_amp * VALUE;
            Dust += ((rt_adc_read(adc_dev, Dust_Motor_channel) * VDD / 4095) - 0.5 * VDD) / Millivolts_per_amp * VALUE;
        }

        Grinding1 /= NUM;
        Grinding2 /= NUM;
        Vacuuming /= NUM;
        Dust /= NUM;



//        rt_kprintf("Grinding1=%f,Grinding2=%f,Vacuuming=%f,Dust=%f\n",Grinding1,Grinding2,Vacuuming,Dust);
        rt_thread_mdelay(5);
    }
}


int data_class_thread_init(void)
{
    rt_err_t ret = RT_EOK;
    rt_thread_t tid = RT_NULL;

    /* 查找光电编码器 */
    pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);

    if (pulse_encoder_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME);
        return ret;
    }
    else{

        rt_kprintf("open %s device success!\n", PULSE_ENCODER_DEV_NAME);
    }


    /* 查找ADC设备 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }
    /* 使能设备 */
    ret = rt_adc_enable(adc_dev, Pressure_sensor1_channel);  /*压力传感器*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Pressure_sensor1_channel);
    ret = rt_adc_enable(adc_dev, Pressure_sensor2_channel);  /*压力传感器*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Pressure_sensor2_channel);
    ret = rt_adc_enable(adc_dev, Pressure_sensor3_channel);  /*压力传感器*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Pressure_sensor3_channel);
    ret = rt_adc_enable(adc_dev, Temp_sensor1_channel);  /*温度传感器*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Temp_sensor1_channel);
    ret = rt_adc_enable(adc_dev, Temp_sensor2_channel);  /*温度传感器*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Temp_sensor2_channel);

    ret = rt_adc_enable(adc_dev, Grinding_Motor1_channel);  /*一号打磨电机*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Grinding_Motor1_channel);
    ret = rt_adc_enable(adc_dev, Grinding_Motor2_channel);  /*二号打磨电机*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Grinding_Motor2_channel);
    ret = rt_adc_enable(adc_dev, Vacuuming_Motor_channel);  /*吸尘电机*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Vacuuming_Motor_channel);
    ret = rt_adc_enable(adc_dev, Dust_Motor_channel);  /*扫灰电机*/
    if (ret == RT_EOK)
        rt_kprintf("adc ch%d enable success!\n",Dust_Motor_channel);




    /* 创建线程1 */
    tid = rt_thread_create("thread1",
                           thread1_entry, RT_NULL,
                           THREAD_STACK_SIZE,
                           THREAD_PRIORITY,
                           THREAD_TIMESLICE);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    MLX90614_SMBus_Init();
    return RT_EOK;
}





INIT_APP_EXPORT(data_class_thread_init);



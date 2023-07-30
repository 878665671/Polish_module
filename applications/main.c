/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-07     RT-Thread    first version
 */



#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <board.h>
#include "app_data_class.h"
#include "app_control_class.h"
#include "app_uart2.h"
#include "math.h"

#define Adaptive_THREAD_PRIORITY         23
#define Adaptive_THREAD_STACK_SIZE       2048
#define Adaptive_THREAD_TIMESLICE        5


#define BUF_SIZE    22 /*发送数据数量*/

#define DOW_VPY_SET 100   /* 自适应下压工作阈值*/
#define STEEP_RATED      /* 额定转速 */
#define TEMP_RATED    1000   /* 额定温度 */
#define CURRENT_RATED  1000 /* 额定电流 */
#define KPA_RATED 3000   /* 额定压力 */

static rt_thread_t Adaptive_tid = RT_NULL;
static rt_thread_t Control_tid = RT_NULL;
static rt_thread_t Data_tid = RT_NULL;


static rt_uint16_t Dow_Value = 800; /*下压力度*/

static int thread_sample(void);
static int control_data_thread_init(void);
static void Grinding_function_ON_OFF(rt_uint8_t isEnable);




/*************************************************************
** Function name:       float_to_2byte
** Descriptions:        float拆分成两个八位整型
** Input parameters:
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
static void float_to_2byte(float temp,rt_uint8_t *byte1,rt_uint8_t *byte2)
{

    *byte1 = (uint8_t)temp;
    *byte2 =  (uint16_t)(temp * 100) % 100;
}


void split_uint16(rt_uint16_t val, rt_uint8_t *high_byte, rt_uint8_t *low_byte)
{
    *high_byte = (val >> 8) & 0xFF;
    *low_byte = val & 0xFF;
}


rt_uint8_t sum_and_checksum(rt_uint8_t arr[], rt_uint8_t len) {
    rt_uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += arr[i];
    }
    return checksum;
}

/* 自适应下压线程的入口函数 */
static void Adaptive_downward_entry(void *parameter)
{
    Down_pressure_motor(2); /*电机下压*/
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 50000, 50000 / 2);//20k
    rt_thread_mdelay(2000);
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 66666, 66666 / 2);//15k
    rt_thread_mdelay(2000);
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 100000, 100000 / 2);//10k
    while(kpa_value < 1);
    Down_pressure_motor(0); /*电机停止*/
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 100000, 100000);//0k

    while(1)
    {
        if(kpa_value >= KPA_RATED)
        {
            rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 100000, 100000 / 2);//10k

            continue;
        }

        if(kpa_value < Dow_Value) /*如果压力传感器值小于设置值*/
        {
            if((Dow_Value - kpa_value) < DOW_VPY_SET){     /*如果差一点点不调整*/
                Down_pressure_motor(0); /*电机停止*/
                continue;
            }

            else if ((Dow_Value - kpa_value) < DOW_VPY_SET + 200){
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 666666, 666666 / 2);//1.5k
                Down_pressure_motor(2); /*电机下压*/
            }
            else {
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 333333, 333333 / 2);//3k
                Down_pressure_motor(2); /*电机下压*/
            }

        }

        if(kpa_value > Dow_Value) /*如果压力传感器值大于设置值*/
        {
//            rt_pin_write(DIR_PIN, DIR_PIN_UP);
            if((kpa_value - Dow_Value) < DOW_VPY_SET){     /*如果差一点点不调整*/
                Down_pressure_motor(0); /*电机停止*/
                continue;
            }
            else if((kpa_value - Dow_Value) < DOW_VPY_SET + 200){
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 200000, 200000 / 2);//5k
                Down_pressure_motor(1); /*电机上升*/
            }
            else {
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 125000, 125000 / 2);//8k
                Down_pressure_motor(1); /*电机上升*/
            }
        }
        rt_thread_mdelay(20);

    }
}


/* 控制指令处理入口函数 */
static void Control_main(void *parameter)
{
    uint8_t high_byte;     // 高8位
    uint8_t low_byte;      // 低8位

    while (1)
    {
        /*阻塞式处理信号*/
        rt_sem_take(sem_write, RT_WAITING_FOREVER);


        if((data[0] != FRAME_HEAD) || (data[1] != 0x52)) /*如果不是帧头或者功能码继续阻塞接收*/
           continue;
//        rt_kprintf("*************************\r\n");
//        rt_kprintf("data[0] = %x,data[1] = %x,data[2] = %x\r\n",data[0],data[1],data[2]);
//        rt_kprintf("data[5] = %x,data[6] = %x,data[7] = %x,data[8] = %x,data[9] = %x\r\n",data[5],data[6],data[7],data[8],data[9]);
//
//        rt_kprintf("*************************\r\n");
        high_byte = data[3];
        low_byte = data[4];

//        rt_kprintf("data[3] = %x,data[4] = %x \r\n",data[3],data[4]);
        Dow_Value = ((uint16_t)high_byte << 8) | low_byte;  // 将高8位左移8位，再与低8位相或
        if (Dow_Value >1200) Dow_Value = 1200;

//        rt_kprintf("Dow_Value = %d \r\n",Dow_Value);

        if(data[2] == 0xA1)
        {
//            rt_kprintf("启用自适应打磨 \r\n");
            Grinding_function_ON_OFF(1);       /*开启自适应打磨功能*/
        }
        else if(data[2] == 0xA0)
        {
//            rt_kprintf("禁用自适应打磨 \r\n");
            Grinding_function_ON_OFF(0);       /*关闭自适应打磨功能*/

            if(data[9] == 0xA1)
            {
//                rt_kprintf("模块初始化 \r\n");
                Mechanical_init();         /*初始化*/
            }
            else
            {
                if (data[8] == 0xA1)                /*下压电机向上*/
                {
//                    rt_kprintf("下压电机上升 \r\n");
                    Down_pressure_motor(1);
                }
                if(data[8] == 0xA0)
                {
//                    rt_kprintf("下压电机停止 \r\n");
                    Down_pressure_motor(0);         /*下压电机停止*/
                }

                if(data[8] == 0xA2)

                {
//                    rt_kprintf("下压电机下压 \r\n");
                    Down_pressure_motor(2);         /*下压电机向下*/
                }

                if (data[7] == 0xA1)                /*开启扫灰电机*/
                {
//                    rt_kprintf("开启扫灰电机 \r\n");
                        Dust_ON_OFF(1);
                }
                if(data[7] == 0xA0)            /*关闭扫灰电机*/
                {
//                    rt_kprintf("关闭扫灰电机 \r\n");
                        Dust_ON_OFF(0);
                }

                if (data[6] == 0xa1)                /*开启吸尘电机*/
                {
//                    rt_kprintf("开启吸尘电机 \r\n");
                        Vacuuming_ON_OFF(1);
                }
                if(data[6] == 0xa0)            /*关闭吸尘电机*/
                {
//                    rt_kprintf("关闭吸尘电机 \r\n");
                        Vacuuming_ON_OFF(0);
                }

                if (data[5] == 0xA1)                /*开启打磨电机*/
                {
//                    rt_kprintf("开启打磨电机 \r\n");
                        Grinding_ON_OFF(1);
                }
                if(data[5] == 0xA0)            /*关闭打磨电机*/
                {
//                    rt_kprintf("关闭打磨电机 \r\n");
                        Grinding_ON_OFF(0);
                }
            }
        }
    }
}

//        if(data[9] == 0xA1)
//        {
//            data[2] = 0x00;
//            data[5] = 0x00;
//            data[6] = 0x00;
//            data[7] = 0x00;
//            data[8] = 0x00;
//
//            rt_kprintf("data[9] == 0xA1\r\n");
//            Mechanical_init();         /*初始化*/
//        }
//        else if(data[2] == 0xA1)
//        {
//            data[5] = 0x00;
//            data[6] = 0x00;
//            data[7] = 0x00;
//            data[8] = 0x00;
//            data[9] = 0x00;
//
//            Grinding_function_ON_OFF(1);       /*开启自适应打磨功能*/
//            rt_kprintf("data[2] == 0xA1\r\n");
//        }
//        else if(data[2] == 0xA0 || data[2] != 0xA1)            /*关闭自适应打磨功能*/
//        {
//            rt_kprintf("data[2] == 0xA0\r\n");
//            Grinding_function_ON_OFF(0);
//
//            if (data[8] == 0xA1)                /*下压电机向上*/
//            {
//                rt_kprintf("data[8] == 0xA1\r\n");
//                Down_pressure_motor(1);
//            }
//            if(data[8] == 0xA0)
//            {
//                rt_kprintf("data[8] == 0xA0\r\n");
//                Down_pressure_motor(0);         /*下压电机停止*/
//            }
//
//            if(data[8] == 0xA2)
//
//            {
//                rt_kprintf("data[8] == 0xA2\r\n");
//                Down_pressure_motor(2);         /*下压电机向下*/
//            }
//            if (data[5] == 0xA1)                /*开启打磨电机*/
//            {
//                rt_kprintf("data[5] == 0xA1\r\n");
//                    Grinding_ON_OFF(1);
//            }
//            if(data[5] == 0xA0)            /*关闭打磨电机*/
//            {
//                rt_kprintf("data[5] == 0xA0\r\n");
//                    Grinding_ON_OFF(0);
//            }
//
//            if (data[6] == 0xa1)                /*开启吸尘电机*/
//            {
//                rt_kprintf("data[6] == 0xA1\r\n");
//                    Vacuuming_ON_OFF(1);
//            }
//            if(data[6] == 0xa0)            /*关闭吸尘电机*/
//            {
//                rt_kprintf("data[6] == 0xA0\r\n");
//                    Vacuuming_ON_OFF(0);
//            }
//
//            if (data[7] == 0xA1)                /*开启扫灰电机*/
//            {
//                rt_kprintf("data[7] == 0xA1\r\n");
//                    Dust_ON_OFF(1);
//            }
//            if(data[7] == 0xA0)            /*关闭扫灰电机*/
//            {
//                rt_kprintf("data[7] == 0xA0\r\n");
//                    Dust_ON_OFF(0);
//            }

//
///* 控制指令处理入口函数 */
//static void Control_main(void *parameter)
//{
//    uint8_t high_byte;     // 高8位
//    uint8_t low_byte;      // 低8位
//
//
//
//    while (1)
//    {
//        /*阻塞式处理信号*/
//        rt_sem_take(sem_write, RT_WAITING_FOREVER);
//
//
//        if((data[0] != FRAME_HEAD) || (data[1] != 0x52)) /*如果不是帧头或者功能码继续阻塞接收*/
//           continue;
//        rt_kprintf("*************************\r\n");
//        rt_kprintf("data[0] = %x,data[1] = %x,data[2] = %x\r\n",data[0],data[1],data[2]);
//        rt_kprintf("data[5] = %x,data[6] = %x,data[7] = %x,data[8] = %x,data[9] = %x\r\n",data[5],data[6],data[7],data[8],data[9]);
//
//        rt_kprintf("*************************\r\n");
//        high_byte = data[3];
//        low_byte = data[4];
//        Dow_Value = ((uint16_t)high_byte << 8) | low_byte;  // 将高8位左移8位，再与低8位相或
//
//
//        if (data[2] == 0xA1)                /*开启自适应打磨功能*/
//       {
//           Grinding_function_ON_OFF(1);
//           rt_kprintf("data[2] == 0xA1\r\n");
//       }
//       if(data[2] == 0xA0)            /*关闭自适应打磨功能*/
//       {
//           rt_kprintf("data[2] == 0xA0\r\n");
//           Grinding_function_ON_OFF(0);
//       }
//       if (data[2] == 0xA0 && data[5] == 0xA1)                /*开启打磨电机*/
//       {
//           rt_kprintf("data[5] == 0xA1\r\n");
//           Grinding_ON_OFF(1);
//       }
//       if(data[2] == 0xA0 && data[5] == 0xA0)            /*关闭打磨电机*/
//       {
//           rt_kprintf("data[5] == 0xA0\r\n");
//           Grinding_ON_OFF(0);
//       }
//
//       if (data[2] == 0xA0 && data[6] == 0xA1)                /*开启吸尘电机*/
//       {
//           rt_kprintf("data[6] == 0xA1\r\n");
//           Vacuuming_ON_OFF(1);
//       }
//       if(data[2] == 0xA0 && data[6] == 0xA0)            /*关闭吸尘电机*/
//       {
//           rt_kprintf("data[6] == 0xA0\r\n");
//           Vacuuming_ON_OFF(0);
//       }
//
//       if (data[2] == 0xA0 && data[7] == 0xA1)                /*开启扫灰电机*/
//       {
//           rt_kprintf("data[7] == 0xA1\r\n");
//
//           Dust_ON_OFF(1);
//       }
//       if(data[2] == 0xA0 && data[7] == 0xA0)            /*关闭扫灰电机*/
//       {
//           rt_kprintf("data[7] == 0xA0\r\n");
//
//           Dust_ON_OFF(0);
//       }
//
//       if (data[9] == 0xA0 && data[2] == 0xA0 && data[8] == 0xA1)                /*下压电机向上*/
//       {
//           rt_kprintf("data[8] == 0xA1\r\n");
//           Down_pressure_motor(1);
//       }
//       if(data[9] == 0xA0 && data[2] == 0xA0 && data[8] == 0xA0)
//       {
//           rt_kprintf("data[8] == 0xA0\r\n");
//           Down_pressure_motor(0);         /*下压电机停止*/
//       }
//
//       if(data[9] == 0xA0 && data[2] == 0xA0 && data[8] == 0xA2)
//       {
//           rt_kprintf("data[8] == 0xA2\r\n");
//           Down_pressure_motor(2);         /*下压电机向下*/
//       }
//
//       if(data[2] == 0xA0 && data[9] == 0xA1)
//       {
//           rt_kprintf("data[9] == 0xA1\r\n");
//           Down_pressure_motor(1);         /*下压电机回零点*/
//       }
//
//       if(data[2] == 0xA0 && data[9] == 0xA0)
//       {
//           rt_kprintf("data[9] == 0xA0\r\n");
//           Down_pressure_motor(0);         /*下压电机停止回零点*/
//       }
//    }
//}




/* 数据指令处理入口函数 */
static void Data_main(void *parameter)
{
    uint8_t buf[BUF_SIZE] = {0x55,0x51};
    while (1)
    {
        /*非阻塞式处理信号*/
        if(RT_EOK == rt_sem_take(sem_read, RT_WAITING_NO))
        {
//            rt_kprintf("buf[0] = %x,buf[1] = %x,buf[2] = %x,buf[3] = %x,buf[4] = %x\r\n",buf[0],buf[1],buf[2],buf[3],buf[4]);
//            rt_kprintf("buf[5] = %x,buf[6] = %x,buf[7] = %x,buf[8] = %x,buf[9] = %x\r\n",buf[5],buf[6],buf[7],buf[8],buf[9]);
//            rt_kprintf("buf[10] = %x,buf[11] = %x,buf[12] = %x,buf[13] = %x,buf[14] = %x\r\n",buf[10],buf[11],buf[12],buf[13],buf[14]);
//            rt_kprintf("buf[15] = %x,buf[16] = %x,buf[17] = %x,buf[18] = %x,buf[19] = %x,buf[20] = %x,buf[21] = %x\r\n",buf[15],buf[16],buf[17],buf[18],buf[19],buf[20],buf[21]);
           rt_thread_mdelay(1000);
            RS485_TX;
//            rt_kprintf("send num = %d\r\n",rt_device_write(uart2_handle, 0, buf, BUF_SIZE));
            rt_device_write(uart2_handle, 0, buf, BUF_SIZE);

            RS485_RX;
        }


        split_uint16(kpa_value, &buf[3], &buf[2]); /*将压力值的高8位和低8位放入数组*/
//        float_to_2byte(temp, &buf[4], &buf[5]);     /*温度*/
        split_uint16((rt_uint16_t)temp, &buf[5], &buf[4]); /*将温度的高8位和低8位放入数组*/
        split_uint16((rt_uint16_t)speed,&buf[7], &buf[6]);       /*转速*/
//        float_to_2byte(Grinding1, &buf[8], &buf[9]);     /*一号打磨电机电流*/
//        float_to_2byte(Grinding2, &buf[10], &buf[11]);   /*二号打磨电机电流*/
//        Grinding1 = Grinding2 + Grinding1;
//        Grinding2 = Grinding2 + Grinding1;
//        if(Grinding1 >= Grinding2)
//            Grinding2 = Grinding1;
//        else if(Grinding2 >= Grinding2)
//            Grinding1 = Grinding2;
        split_uint16(Grinding1,&buf[9], &buf[8]);       /*一号打磨电机电流*/
        split_uint16(Grinding2,&buf[11], &buf[10]);       /*二号打磨电机电流*/
//        float_to_2byte(Dust, &buf[12], &buf[13]);   /*扫灰电机电流*/
//        float_to_2byte(Vacuuming, &buf[14], &buf[15]);   /*吸尘电机电流*/
        split_uint16(Dust * 100,&buf[13], &buf[12]);       /*扫灰电机电流*/
        split_uint16(Vacuuming,&buf[15], &buf[14]);       /*扫灰电机电流*/


//        buf[2] = 0x37;
//        buf[3] = 0x37;
//        buf[4] = 0x37;
//        buf[5] = 0x37;
//        buf[6] = 0x37;
//        buf[7] = 0x37;
//        buf[8] = 0x37;
//        buf[9] = 0x37;
//        buf[10] = 0x37;
//        buf[11] = 0x37;
//        buf[12] = 0x37;
//        buf[13] = 0x37;
//        buf[14] = 0x37;
//        buf[15] = 0x37;





        if(KEY_PIN_FALG == rt_pin_read(KEY_PIN_NUM))
            buf[16] = 0xA1;
        else if(KEY_PIN_FALG != rt_pin_read(KEY_PIN_NUM)){
            buf[16] = 0xA0;
        }
        buf[17] = Grinding_Motor1_State();
        buf[17] = Grinding_Motor2_State();
        buf[18] = Vacuuming_Motor_State();
        buf[19] = Dust_Motor_State();
        buf[20] = Down_Motor_State();
        buf[21] = sum_and_checksum(buf, BUF_SIZE - 1);

//        rt_kprintf("**********************************************\n");
        rt_thread_mdelay(10);
    }
}



/*************************************************************
** Function name:       Adaptive_downward_ON_OFF
** Descriptions:        自适应下压
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
static void Adaptive_downward_ON_OFF(rt_uint8_t isEnable)
{
    if(isEnable == 1){
        if(Adaptive_tid->stat == RT_THREAD_INIT || (Adaptive_tid->stat == RT_THREAD_CLOSE))
        {
//            rt_pwm_set_period(pwm_dev, PWM_DEV_CHANNEL, 200000);
            thread_sample();
            rt_thread_startup(Adaptive_tid);
        }
    }
    else if (isEnable == 0) {
        if((Adaptive_tid->stat) != RT_THREAD_INIT) /* 如果线程处于初始化状态或者被删除过不执行 */
        {
                                                /*线程在没有初始化时他的状态是RT_THREAD_INIT，如过初始化后被delet删除处于RT_THREAD_CLOSE */
            if((Adaptive_tid->stat) != RT_THREAD_CLOSE)
            {
                rt_thread_delete(Adaptive_tid);
//                rt_pwm_set_period(pwm_dev, PWM_DEV_CHANNEL, 50000);
            }
        }
    }
}
/*************************************************************
** Function name:       Grinding_function_ON_OFF
** Descriptions:        开启关闭自适应打磨功能
** Input parameters:    isEnable：1开启、0关闭
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void Grinding_function_ON_OFF(rt_uint8_t isEnable)
{
    static rt_uint8_t state;
    if(state == isEnable)
        return;
    state = isEnable;
    if(isEnable == 1){
        Mechanical_init();           /* 机械初始化 */
        Vacuuming_ON_OFF(1);    /* 开启吸尘电机继电器 */
        Dust_ON_OFF(1);         /* 开启扫灰电机继电器 */
        rt_thread_mdelay(2500);
        Grinding_ON_OFF(1);     /* 开启打磨电机继电器 */
        Adaptive_downward_ON_OFF(1); /* 启动自适应下压 */
    }
    if (isEnable == 0) {
        Adaptive_downward_ON_OFF(0); /* 关闭自适应下压 */
        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 33333, 33333 / 2);//30k
        Mechanical_init();           /* 机械初始化 */
//        Down_pressure_motor(0);         /*下压电机停止*/
    }
}
void test_on()
{
    Grinding_function_ON_OFF(1);
}
MSH_CMD_EXPORT(test_on,test_on);

void test_off()
{
    Grinding_function_ON_OFF(0);
}
MSH_CMD_EXPORT(test_off,test_off);

int main(void)
{
    /* 初始化程序 */
    Mechanical_init();   /*先把下压电机上升到原点*/
    uart_sample();       /*初始化与上位机通讯端口*/
    control_data_thread_init(); /*初始化控制与数据处理线程*/

    while (1)
    {
        if(KEY_PIN_FALG == PIN_FLAG())
        {                               /* 如果下压电机上升到极限停止下压电机 */
            Down_pressure_motor(0);
            Down_pressure_motor_State(1);
            rt_kprintf("The Dowmotor rises to its limit!\n");
            while(KEY_PIN_FALG == PIN_FLAG())  /* 防止重复打印 */
            {
//                rt_kprintf("The Dowmotor rises to its limit!\n");
                rt_thread_mdelay(20);
            }
        }
        else
            Down_pressure_motor_State(0);

        if(kpa_value > KPA_RATED)
        {
            Down_pressure_motor(0); /*如果下压力大于设定值极限下压电机停止*/
            Down_pressure_motor_State(2);
            Grinding_function_ON_OFF(0);
            rt_kprintf("The Dowmotor drops to its limit!\n");
        }
        else
            Down_pressure_motor_State(0);
        if(temp > TEMP_RATED)
        {
            Grinding_ON_OFF(0); /*如果转速大于设定值极限打磨电机停止*/
            rt_kprintf("The Grinding Temp to its limit!\n");
        }

        if(Grinding1 >= CURRENT_RATED || Grinding2 >= CURRENT_RATED)
        {
            Grinding_ON_OFF(0); /*如果打磨电机电流大于设定值极限打磨电机停止*/
            rt_kprintf("The Grinding Current to its limit!,Grinding1 Current %f.Grinding2 Current %f\r\n",Grinding1,Grinding2);
        }
        rt_thread_mdelay(20);
    }
    return RT_EOK;
}


static int thread_sample(void)
{

    Adaptive_tid = rt_thread_create("Adaptive_downward",
                            Adaptive_downward_entry, RT_NULL,
                            Adaptive_THREAD_STACK_SIZE,
                            Adaptive_THREAD_PRIORITY, Adaptive_THREAD_TIMESLICE);

    if (Adaptive_tid != RT_NULL)
       rt_kprintf("Adaptive_downward thread init success!\n");
    return 0;
}
MSH_CMD_EXPORT(thread_sample,thread_sample);
static int control_data_thread_init(void)
{
    Control_tid = rt_thread_create("Control_thread",
                            Control_main, RT_NULL,
                            2048,
                            10, 100);

    if (Control_tid != RT_NULL)
       rt_kprintf("Control_tid thread init success!\n");

    Data_tid = rt_thread_create("Data_thread",
                            Data_main, RT_NULL,
                            2048,
                            19, 100);

    if (Data_tid != RT_NULL)
       rt_kprintf("Data_tid thread init success!\n");
    rt_thread_startup(Control_tid);
    rt_thread_startup(Data_tid);
    return 0;
}

/* END main.c*/
//**************************************************************************************************************//
//**************************************************************************************************************//
//**************************************************************************************************************//
//**************************************************************************************************************//
//**************************************************************************************************************//





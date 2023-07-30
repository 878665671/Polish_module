/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-30     CatUncle       the first version
 */
#ifndef APPLICATIONS_APP_UART2_H_
#define APPLICATIONS_APP_UART2_H_

/* 是否存在控制引脚 */
#define RS485_WP_PIN


#define FRAME_NUM 11   /* 接收帧数 */
#define FRAME_HEAD 0x55  /* 帧头 */

#define RS485_WP    GET_PIN(D, 4)
#define RS485_TX    rt_pin_write(RS485_WP, PIN_HIGH)
#define RS485_RX    rt_pin_write(RS485_WP, PIN_LOW)


extern rt_uint8_t data[FRAME_NUM];
extern rt_sem_t sem_write,sem_read;

extern rt_device_t uart2_handle;
int uart_sample();
#endif /* APPLICATIONS_APP_UART2_H_ */

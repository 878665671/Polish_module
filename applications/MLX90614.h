/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-08     yy       the first version
 */
#ifndef APPLICATIONS_MLX90614_H_
#define APPLICATIONS_MLX90614_H_


#include <board.h>
#define ACK  0
#define NACK 1  //不应答或否定的应答
#define SA              0x00 //从机地址，单个MLX90614时地址为0x00,多个时地址默认为0x5a
#define RAM_ACCESS      0x00 //RAM access command
#define EEPROM_ACCESS   0x20 //EEPROM access command
#define RAM_TOBJ1       0x07 //To1 address in the eeprom


#define MLX_SCL_PINS GET_PIN(B,5)
#define MLX_SDA_PINS GET_PIN(B,6)


#define SMBUS_SDA_PIN  MLX_SDA_PINS
#define SMBUS_SDA_INPUT()  rt_pin_mode(SMBUS_SDA_PIN, PIN_MODE_INPUT)
#define SMBUS_SDA_OUTPUT() rt_pin_mode(SMBUS_SDA_PIN, PIN_MODE_OUTPUT);
#define  SMBUS_SCK_H   rt_pin_write(MLX_SCL_PINS, PIN_HIGH)
#define  SMBUS_SCK_L   rt_pin_write(MLX_SCL_PINS, PIN_LOW)
#define  SMBUS_SDA_H   rt_pin_write(MLX_SDA_PINS, PIN_HIGH)
#define  SMBUS_SDA_L   rt_pin_write(MLX_SDA_PINS, PIN_LOW)


#define SMBUS_DELAY_1US rt_hw_us_delay(1)
#define SMBUS_DELAY_2US rt_hw_us_delay(2)
#define SMBUS_DELAY_3US rt_hw_us_delay(3)
#define SMBUS_DELAY_4US rt_hw_us_delay(4)
#define SMBUS_DELAY_5US rt_hw_us_delay(5)
#define SMBUS_DELAY_6US rt_hw_us_delay(6)

void SMBus_StartBit(void);
void SMBus_StopBit(void);
void SMBus_SendBit(unsigned char);
unsigned char SMBus_SendByte(unsigned char);
unsigned char SMBus_ReceiveBit(void);
unsigned char SMBus_ReceiveByte(unsigned char);
void SMBus_Delay(unsigned int);
void MLX90614_SMBus_Init(void);
unsigned int SMBus_ReadMemory(unsigned char, unsigned char);
unsigned char PEC_Calculation(unsigned char*);
float SMBus_ReadTemp(void);    //获取温度值


#endif /* APPLICATIONS_MLX90614_H_ */

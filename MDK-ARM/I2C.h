/**************************************************************************//**
 * @file     CTR_I2C.h
 * @Deverloper	lochoang360			    
 *           
 * @version  V0.1
 * @date     14. Oct 2016
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2016 LINH KIEN 3M*/

#ifndef _I2C_
#define _I2C_
#include "stm32f1xx_hal.h"
#include "stdbool.h"
#define SCL_H         GPIOB->BSRR = GPIO_PIN_8
#define SCL_L         GPIOB->BRR  = GPIO_PIN_8 
   
#define SDA_H         GPIOB->BSRR = GPIO_PIN_9
#define SDA_L         GPIOB->BRR  = GPIO_PIN_9

#define SCL_read      GPIOB->IDR  & GPIO_PIN_8
#define SDA_read      GPIOB->IDR  & GPIO_PIN_9

void CTR_I2C_GPIO_Init(void);
void CTR_I2C_Delay(void);
void CTR_I2C_Delay5ms(void);
bool CTR_I2C_Start(void);
void CTR_I2C_Stop(void);
void CTR_I2C_Ack(void);
void CTR_I2C_NoAck(void);
bool CTR_I2C_WaitAck(void);
void CTR_I2C_SendByte(uint8_t SendByte) ;
unsigned char CTR_I2C_RadeByte(void) ;
bool CTR_Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char CTR_Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
#endif


#include "I2C.h"

void CTR_I2C_GPIO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 
  GPIO_InitStructure.Pin =  GPIO_PIN_6;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.Pin =  GPIO_PIN_7;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

}
/*delay for I2C*/
void CTR_I2C_Delay(void)
{
		
   uint8_t i=30; 
   while(i) 
   { 
     i--; 
   }  
}

void CTR_I2C_Delay5ms(void)
{
		
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}

bool CTR_I2C_Start(void)
{
	SDA_H;
	SCL_H;
	CTR_I2C_Delay();
	if(!SDA_read)return false;	
	SDA_L;
	CTR_I2C_Delay();
	if(SDA_read) return false;	
	SDA_L;
	CTR_I2C_Delay();
	return true;
}

void CTR_I2C_Stop(void)
{
	SCL_L;
	CTR_I2C_Delay();
	SDA_L;
	CTR_I2C_Delay();
	SCL_H;
	CTR_I2C_Delay();
	SDA_H;
	CTR_I2C_Delay();
} 

void CTR_I2C_Ack(void)
{	
	SCL_L;
	CTR_I2C_Delay();
	SDA_L;
	CTR_I2C_Delay();
	SCL_H;
	CTR_I2C_Delay();
	SCL_L;
	CTR_I2C_Delay();
}   

void CTR_I2C_NoAck(void)
{	
	SCL_L;
	CTR_I2C_Delay();
	SDA_H;
	CTR_I2C_Delay();
	SCL_H;
	CTR_I2C_Delay();
	SCL_L;
	CTR_I2C_Delay();
} 

bool CTR_I2C_WaitAck(void) 	 
{
	SCL_L;
	CTR_I2C_Delay();
	SDA_H;			
	CTR_I2C_Delay();
	SCL_H;
	CTR_I2C_Delay();
	if(SDA_read)
	{
      SCL_L;
	  CTR_I2C_Delay();
      return false;
	}
	SCL_L;
	CTR_I2C_Delay();
	return true;
}

void CTR_I2C_SendByte(uint8_t SendByte) 
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        CTR_I2C_Delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        CTR_I2C_Delay();
		SCL_H;
        CTR_I2C_Delay();
    }
    SCL_L;
}  

unsigned char CTR_I2C_RadeByte(void)  
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      CTR_I2C_Delay();
	  SCL_H;
      CTR_I2C_Delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 

bool CTR_Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     
{
  	if(!CTR_I2C_Start())return false;
    CTR_I2C_SendByte(SlaveAddress);   
    if(!CTR_I2C_WaitAck()){CTR_I2C_Stop(); return false;}
    CTR_I2C_SendByte(REG_Address );   //
    CTR_I2C_WaitAck();	
    CTR_I2C_SendByte(REG_data);
    CTR_I2C_WaitAck();   
    CTR_I2C_Stop(); 
    CTR_I2C_Delay5ms();
    return true;
}
int8_t test =0;
unsigned char CTR_Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{  
	unsigned char REG_data;     	
	if(!CTR_I2C_Start())return false;
    CTR_I2C_SendByte(SlaveAddress); 
    if(!CTR_I2C_WaitAck()){CTR_I2C_Stop();test=1; return false;}
    CTR_I2C_SendByte((uint8_t) REG_Address);  
    CTR_I2C_WaitAck();
    CTR_I2C_Start();
    CTR_I2C_SendByte(SlaveAddress+1);
    CTR_I2C_WaitAck();

	  REG_data= CTR_I2C_RadeByte();
    CTR_I2C_NoAck();
    CTR_I2C_Stop();    
	return REG_data;
}		
/*end of file*/
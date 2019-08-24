#include "MPU6050.h"
unsigned char CTR_MPU6050_BUF[8];
void CTR_Init_MPU6050(void)
{

   CTR_Single_Write(CTR_MPU6050_Addr,CTR_PWR_M, 0x80);   //
   CTR_Single_Write(CTR_MPU6050_Addr,CTR_SMPL, 0x07);    //
   CTR_Single_Write(CTR_MPU6050_Addr,CTR_DLPF, 0x1E);    //±2000°
   CTR_Single_Write(CTR_MPU6050_Addr,CTR_INT_C, 0x00 );  //
   CTR_Single_Write(CTR_MPU6050_Addr,CTR_PWR_M, 0x00);   //
}
float * CTR_READ_MPU6050(void)
{
	
	static float mpuData[4];
   CTR_MPU6050_BUF[0]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_GX_L); 
   CTR_MPU6050_BUF[1]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_GX_H);
   mpuData[0]=	(CTR_MPU6050_BUF[1]<<8)|CTR_MPU6050_BUF[0];
   mpuData[0]/=16.4; 						   

   CTR_MPU6050_BUF[2]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_GY_L);
   CTR_MPU6050_BUF[3]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_GY_H);
   mpuData[1]=	(CTR_MPU6050_BUF[3]<<8)|CTR_MPU6050_BUF[2];
   mpuData[1]/=16.4; 						   
   CTR_MPU6050_BUF[4]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_GZ_L);
   CTR_MPU6050_BUF[5]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_GZ_H);
   mpuData[2]=	(CTR_MPU6050_BUF[5]<<8)|CTR_MPU6050_BUF[4];
   mpuData[2]/=16.4; 					       

   CTR_MPU6050_BUF[6]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_TMP_L); 
   CTR_MPU6050_BUF[7]=CTR_Single_Read(CTR_MPU6050_Addr,CTR_TMP_H); 
   mpuData[3]=(CTR_MPU6050_BUF[7]<<8)|CTR_MPU6050_BUF[6];
   mpuData[3] = 35+ ((double) (mpuData[3] + 13200)) / 280;
	return mpuData;
}

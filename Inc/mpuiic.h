#ifndef __MPUIIC_H
#define __MPUIIC_H
   		   
//SDA --------- PA4
//SCL --------- PA5
#include "main.h"					 


//更换引脚，记得初始化函数也要更改
//IO方向设置
#define MPU_SDA_IN()  {GPIOA->CRL&=0XFFF0FFFF;GPIOA->CRL|=8<<16;}
#define MPU_SDA_OUT() {GPIOA->CRL&=0XFFF0FFFF;GPIOA->CRL|=3<<16                                           ;}

//IO操作函数	 
#define MPU_IIC_SCL    PAout(5) 		//SCL
#define MPU_IIC_SDA    PAout(4) 		//SDA	 
#define MPU_READ_SDA   PAin(4) 		//输入SDA 
					 
////IO方向设置
//#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
//#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

////IO操作函数	 
//#define MPU_IIC_SCL    PBout(10) 		//SCL
//#define MPU_IIC_SDA    PBout(11) 		//SDA	 
//#define MPU_READ_SDA   PBin(11) 		//输入SDA 

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif












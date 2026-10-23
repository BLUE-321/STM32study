#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050.h"
#include <stdint.h>

#define MPU6050_ADDRESS		0xD0

// 加速度寄存器地址
#define MPU6050_ACCEL_XOUT_H	0x3B
#define MPU6050_ACCEL_XOUT_L	0x3C
#define MPU6050_ACCEL_YOUT_H	0x3D
#define MPU6050_ACCEL_YOUT_L	0x3E
#define MPU6050_ACCEL_ZOUT_H	0x3F
#define MPU6050_ACCEL_ZOUT_L	0x40

// 角速度寄存器地址
#define MPU6050_GYRO_XOUT_H		0x43
#define MPU6050_GYRO_XOUT_L		0x44
#define MPU6050_GYRO_YOUT_H		0x45
#define MPU6050_GYRO_YOUT_L		0x46
#define MPU6050_GYRO_ZOUT_H		0x47
#define MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_WHO_AM_I		0x75

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS); // 0xD0，这是写入地址
    MyI2C_RecieveAck();
    MyI2C_SendByte(RegAddress);      // 告诉它我要改哪个寄存器
    MyI2C_RecieveAck();
    MyI2C_SendByte(Data);            // 告诉它改成什么
    MyI2C_RecieveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS); // 0xD0
    MyI2C_RecieveAck();
    MyI2C_SendByte(RegAddress);      // 指向要读的寄存器
    MyI2C_RecieveAck();
    
    MyI2C_Start();                   // 重复起始信号！
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01); // 0xD1，这是读取地址
    MyI2C_RecieveAck();
    
    Data = MyI2C_RecieveByte();      // 读取数据
    MyI2C_SendAck(1);                // 告诉它：我读完了，非应答 (NACK)
    MyI2C_Stop();
    
    return Data;
}

void MPU6050_Init(void)
{
    MyI2C_Init(); // 先把你的 I2C 引脚初始化了
    MPU6050_WriteReg(0x6B, 0x00); // 解除休眠
    MPU6050_WriteReg(0x1C, 0x18); // 配置加速度量程
    MPU6050_WriteReg(0x1B, 0x18); // 配置陀螺仪量程
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
 * @brief 读取加速度和角速度的原始数据
 * 将高8位和低8位拼接成 int16_t 类型
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t AccXH, AccXL, AccYH, AccYL, AccZH, AccZL;
	uint8_t GyroXH, GyroXL, GyroYH, GyroYL, GyroZH, GyroZL;
	
	// 读取加速度 X 轴
	AccXH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	AccXL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (int16_t)((AccXH << 8) | AccXL);
	
	// 读取加速度 Y 轴
	AccYH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	AccYL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (int16_t)((AccYH << 8) | AccYL);
	
	// 读取加速度 Z 轴
	AccZH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	AccZL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (int16_t)((AccZH << 8) | AccZL);
	
	// 读取角速度 X 轴
	GyroXH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	GyroXL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (int16_t)((GyroXH << 8) | GyroXL);
	
	// 读取角速度 Y 轴
	GyroYH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	GyroYL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (int16_t)((GyroYH << 8) | GyroYL);
	
	// 读取角速度 Z 轴
	GyroZH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	GyroZL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (int16_t)((GyroZH << 8) | GyroZL);
}

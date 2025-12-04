#ifndef __JY901S_H
#define __JY901S_H

#include "stm32f10x.h"

// ----------------- 数据结构 -----------------
typedef struct {
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    uint8_t status;
} Gyro_Data_t;

extern Gyro_Data_t Gyro_Data;
extern uint8_t Gyro_UpdateFlag;

// ----------------- 函数声明 -----------------
void Gyro_Init(void);
void Gyro_ParseData(uint8_t *data, uint16_t length);
void Gyro_ProcessByte(uint8_t byte);   // 串口接收字节 → 数据拼接

#endif

#include "JY901S.h"
#include "string.h"
#include "stdio.h"

// ----------------- 全局变量 -----------------
Gyro_Data_t Gyro_Data;
uint8_t Gyro_UpdateFlag = 0;

uint8_t Gyro_RxBuffer[11];
uint8_t Gyro_RxIndex = 0;

// ----------------- 初始化函数 -----------------
void Gyro_Init(void)
{
    memset(&Gyro_Data, 0, sizeof(Gyro_Data_t));

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // TX PA2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RX PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

// ----------------- 拼接帧（每字节进入此函数） -----------------
void Gyro_ProcessByte(uint8_t byte)
{
    if (Gyro_RxIndex == 0 && byte != 0x55)
        return;

    Gyro_RxBuffer[Gyro_RxIndex++] = byte;

    if (Gyro_RxIndex >= 11)
    {
        Gyro_ParseData(Gyro_RxBuffer, 11);
        Gyro_RxIndex = 0;
    }
}

// ----------------- 解析数据帧 -----------------
void Gyro_ParseData(uint8_t *data, uint16_t length)
{
    if (length != 11 || data[0] != 0x55)
        return;

    switch(data[1])
    {
        case 0x53:  // 角度
            Gyro_Data.angle_x = (float)((int16_t)(data[3]<<8 | data[2])) / 32768.0 * 180.0;
            Gyro_Data.angle_y = (float)((int16_t)(data[5]<<8 | data[4])) / 32768.0 * 180.0;
            Gyro_Data.angle_z = (float)((int16_t)(data[7]<<8 | data[6])) / 32768.0 * 180.0;
            Gyro_UpdateFlag = 1;
            break;

        case 0x52:  // 角速度
            Gyro_Data.gyro_x = (float)((int16_t)(data[3]<<8 | data[2])) / 32768.0 * 2000.0;
            Gyro_Data.gyro_y = (float)((int16_t)(data[5]<<8 | data[4])) / 32768.0 * 2000.0;
            Gyro_Data.gyro_z = (float)((int16_t)(data[7]<<8 | data[6])) / 32768.0 * 2000.0;
            break;

        case 0x51:  // 加速度
            Gyro_Data.accel_x = (float)((int16_t)(data[3]<<8 | data[2])) / 32768.0 * 16.0;
            Gyro_Data.accel_y = (float)((int16_t)(data[5]<<8 | data[4])) / 32768.0 * 16.0;
            Gyro_Data.accel_z = (float)((int16_t)(data[7]<<8 | data[6])) / 32768.0 * 16.0;
            break;

        default:
            break;
    }
}



#ifndef DRIVE_STRUCT_H_
#define DRIVE_STRUCT_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned int u32;

/**
 * @brief  Motor structure definition
 */
typedef struct
{
    const u8 NodeID; /* 站ID */

    volatile int ActualPosition; /* 编码器值 */
    volatile int ActualPosition2; /* 编码器值2 */    
    volatile u16 StatusWord;     /* 状态字 */
    volatile u16 StatusWord2;     /* 状态字 */
    volatile u8 EnableStatus; /* 使能状态 */
    volatile u8 EnableStatus2; /* 使能状态2 */
    volatile u8 Fault;          /* 故障状态 */

    volatile int SpeedRatio; /* 速度下发数字量 */
    volatile int SpeedRatio2; /* 速度下发数字量2 */

    volatile int ExpectPositionRatio; /* 期望目标位置 编码器值 */
    volatile int ExpectPositionRatio2; /* 期望目标位置 编码器值 */
    
    long timeStamp; /*!< 单位：ms */

} MotorTypedef;

typedef struct {
    int ActualPosition_L; /* 举升电机编码器值 */
    int ActualPosition_T; /* 旋转电机编码器值 */

    float Height; /* 高度 mm (0-40mm) */
    float Angle; /* 角度 °    顺时针为正 */

} LiftMachineTypedef;

#endif /* MOTOR_H_ */

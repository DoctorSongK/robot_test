#ifndef DUAL_STEER_H_
#define DUAL_STEER_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "../common/struct_drive.h"
#include "../data_out_process/data_out.h"
/*!<AngleFactor 舵輪角度*轉換為對應轉向電機編碼器值因子*/
#define Af (1.0/(2*M_PI)*160*65536)
/*!<OmegaFactor 舵輪角速度°/s轉換成對應轉向電機編碼器值因子*/
#define Of 1.0/360*60*160*17896
/*!<DistanceFactor 行走電機編碼器值轉換成對應距離m因子*/
#define Df 1.0/10000/34.5686*(M_PI*0.25)
/*!<SpeedFactor 速度m/s轉換成行走電機設定值因子*/
#define Sf 1.0/(M_PI*0.25)*60*34.5686*2731
/*!<舵輪到質心距離m */
#define disL 1.0
class Dual_steer
{
public:
    Dual_steer();
    ~Dual_steer();
    
    void SteerWheel_MotorRun(Data_out data);
    void SteerWheel_MotorRun(double front_Velocity,double back_Velocity,double front_Angular,double back_Angular);
 
    MotorTypedef Motor_Run_Front = { .NodeID = 3 };  /* 左行走 */
    MotorTypedef Motor_Turn_Front = { .NodeID = 4 }; /* 右行走 */
    MotorTypedef Motor_Run_Back = { .NodeID = 5 };  /* 左行走 */
    MotorTypedef Motor_Turn_Back = { .NodeID = 6 };
 
private:
    /**
 * @brief Motor Externed
 */


};

#endif /* MOTOR_H_ */

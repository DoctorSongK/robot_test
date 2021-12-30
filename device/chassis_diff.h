#ifndef CHASSIS_DRIVE_H_
#define CHASSIS_DRIVE_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "../common/struct_drive.h"
#include "../data_out_process/data_out.h"

class Chassis_drive
{
public:
    Chassis_drive();
    ~Chassis_drive();
    
    void Diff_MotorRun(Data_out data);
    void Diff_MotorRun(float vl,float vr);
    void Diff_MotorRun_YB(Data_out data);
    void Diff_MotorRun_YB(float vl,float vr);
    MotorTypedef Motor_L = { .NodeID = 5 };  /* 左行走 */
    MotorTypedef Motor_R = { .NodeID = 6 }; /* 右行走 */
 
    MotorTypedef Motor_YB = { .NodeID = 5 }; /* 迎宾机器人伺服 */
private:
    /**
 * @brief Motor Externed
 */
    float _reduce = 9;
    float _ratio = 2731;

};

#endif /* MOTOR_H_ */

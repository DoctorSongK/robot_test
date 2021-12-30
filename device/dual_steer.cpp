/* 步科驱动器CanOpen协议 */

#include "dual_steer.h"
#include <iostream>
#include <math.h>
/**
 * @brief 电机使能
 *
 * @return
 */

Dual_steer::Dual_steer()
{
	;
}

Dual_steer::~Dual_steer()
{

}
void Dual_steer::SteerWheel_MotorRun(Data_out data)
{
    double front_velocity = data.move_info.vel_f_steer;
    double back_velocity = data.move_info.vel_b_steer;///////////////////////////////////////////////////////////////////////////////這裡要和師兄對接
    double front_angular = data.move_info.rot_f_steer;
    double back_angular = data.move_info.rot_b_steer;

    Motor_Run_Front.SpeedRatio = front_velocity*Sf;//轉換成轉每分然後再計算脈衝量
    Motor_Run_Back.SpeedRatio = back_velocity*Sf; 
    Motor_Turn_Front.ExpectPositionRatio = front_angular*Af;
    Motor_Turn_Front.SpeedRatio = 10*Of;//這裡的速度可能需要調
    Motor_Turn_Back.ExpectPositionRatio = back_angular*Af;
    Motor_Turn_Back.SpeedRatio = 10*Of;
}
void Dual_steer::SteerWheel_MotorRun(double front_Velocity,double back_Velocity,double front_Angular,double back_Angular)
{
    Motor_Run_Front.SpeedRatio = front_Velocity*Sf;
    Motor_Run_Back.SpeedRatio = back_Velocity*Sf;
    Motor_Turn_Front.ExpectPositionRatio = front_Angular*Af;
    Motor_Turn_Front.SpeedRatio = 10*Of;//這裡的速度可能需要調
    Motor_Turn_Back.ExpectPositionRatio = back_Angular*Af;
    Motor_Turn_Back.SpeedRatio = 10*Of;
}
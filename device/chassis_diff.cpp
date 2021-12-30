/* 步科驱动器CanOpen协议 */

#include "chassis_diff.h"
#include <iostream>
#include <math.h>
#include "../common/configuration.h"
/**
 * @brief 电机使能
 *
 * @return
 */

Chassis_drive::Chassis_drive()
{
    _reduce = get_configs()->get_float("odom", "axis_reduce", nullptr);
}

Chassis_drive::~Chassis_drive()
{

}
void Chassis_drive::Diff_MotorRun(Data_out data)
{

    float vl, vr, ratio, reduce;
    reduce = _reduce;
    ratio = _ratio;

    vl = data.move_info.vel_l;
    vr = data.move_info.vel_r;

    vr *= 60 / (2 * M_PI ) * ratio * reduce;
    vl *= -60 / (2 * M_PI ) * ratio * reduce;
    Motor_L.SpeedRatio = vl;
    Motor_R.SpeedRatio = vr;
}
void Chassis_drive::Diff_MotorRun(float _vl,float _vr)
{
    float vl, vr,ratio, reduce;
    reduce = _reduce;
    ratio = _ratio;
    vl = _vl;
    vr = _vr;

    vr *= 60 / (2 * M_PI ) * ratio * reduce;
    vl *= -60 / (2 * M_PI ) * ratio * reduce;
    Motor_L.SpeedRatio = vl;
    Motor_R.SpeedRatio = vr;
}
void Chassis_drive::Diff_MotorRun_YB(Data_out data)
{
    float vl, vr, ratio, reduce;
    reduce = 1.0;
    ratio = 1.0;

    vl = data.move_info.vel_l;
    vr = data.move_info.vel_r;

    vr *= 60 / (2 * M_PI ) * ratio * reduce;
    vl *= -60 / (2 * M_PI ) * ratio * reduce;
    Motor_YB.SpeedRatio = (int)vl;
    Motor_YB.SpeedRatio2 = (int)vr;
    //printf("vl=%f,vr=%f, SpeedRatio=%d,SpeedRatio2=%d\n",data.move_info.vel_l,data.move_info.vel_r,Motor_YB.SpeedRatio,Motor_YB.SpeedRatio2);
}
void Chassis_drive::Diff_MotorRun_YB(float _vl,float _vr)
{
    float vl, vr,ratio, reduce;
    reduce = 1;
    ratio = 1;
    vl = _vl;
    vr = _vr;

    vr *= 60 / (2 * M_PI ) * ratio * reduce;
    vl *= -60 / (2 * M_PI ) * ratio * reduce;
    Motor_YB.SpeedRatio = vl;
    Motor_YB.SpeedRatio2 = vr;
    //printf("vl=%f,vr=%f, SpeedRatio=%f,SpeedRatio2=%f\n",_vl,_vr,Motor_YB.SpeedRatio,Motor_YB.SpeedRatio2);
}
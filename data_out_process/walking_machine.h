#ifndef AGV_MODULE_H
#define AGV_MODULE_H

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <list>
#include "../common/struct_drive.h"
#include <Eigen/Core>

#include "../common/pose.h"
#include "../common/capture_point.h"
#include "../common/speed_data.h"
#include "../common/sensor_data.h"
#include "../data_out_process/data_out.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
using namespace Eigen;

class Model_kine  {
public:
    Model_kine();
    ~Model_kine();

    int  robot_control_type;
    double len_b,len_f;/////双舵轮前后轮距离中心的距离
    float alpha_b,alpha_a;////前后舵轮与底盘中心的夹角
    float encoder_per_round;/////the linear motor encoder
    float encoder_per_round_r;////the rotate motoe encoder
    double sin_a,cos_a,sin_b,cos_b;
    float imu_vel_sum;////imu积分的线速度
    float diameter;
    float distance;
    int  back_kine_diff(Speed_data sp_data,st_motion &move_out);
    int  back_kine_dual_steer(Speed_data sp_data,st_motion &move_out);
    int  back_kine(Speed_data sp_data,st_motion &move_out);

    int forward_kine(Position *buf, ImuData2D *imubuf,long delta_t, long delta_fl1, long _fr1,long delta_bl1, long _br1, double delta_theta,double delta_vel);
    int forward_kine(Position *buf, ImuData2D *imubuf,long delta_t, long delta_left, long delta_right, double delta_theta,double delta_vel);
private:
    float zero_turn_angle;
    
    float _half_d = 0.25; /* 輪子到中心距離m   这里要加入文件读取函数 */
    float _radius = 0.072;
    float last_vel_x = 0;
    float last_vel_y = 0;    
	
};

#endif

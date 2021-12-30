#include <map>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include "../common/data_transform.h"
#include "../common/configuration.h"
#include "status_data_dispose.h"
#include "ridar_to_obavoid.h"

Status_dispose::Status_dispose(MsgQueue *queue,  int robot_sts_chan, int radar_obs_chan)
{
    warning_bat = get_configs()->get_int("battery", "warning_low", nullptr);
    use_radar_type = get_configs()->get_int("radar", "use_radar_type", nullptr);
    robot_sts_c = robot_sts_chan;
    radar_obs_c = radar_obs_chan;	
    this->queue = queue;
    queue->add_listener(robot_sts_c, this);
    queue->add_listener(radar_obs_c, this);
    

};

Status_dispose::~Status_dispose() {
    queue->remove_listener(robot_sts_c, this);
    queue->remove_listener(radar_obs_c, this);
};



void Status_dispose::recieve_message(const int channel, char *buf, const int size) {
    if (channel == robot_sts_c)
    {
        Warn_err_data sts_data;
        sts_data.from_char_array(buf, size);
		handle_sts_data(sts_data);
	//std::cout<<"3333333333333333333333"<<" "<<sts_data.ob_3d_sts.ahead_obstacle<<" "<<sts_data.ob_3d_sts.rear_obstacle<<std::endl;
	   //printf("3333333333333333333333333333333333 %d,%d\n",sts_data.ob_3d_sts.ahead_obstacle,sts_data.ob_3d_sts.rear_obstacle); 
    }else if (channel == radar_obs_c && use_radar_type == 0)
    {
        Laser_obs_data obs_data;
        collision_level = obs_data.laser_obs_level;
	}
    else if(channel == radar_obs_c && use_radar_type == 1){
        Laser_obs_3d_data obs_data;
        obs_data.from_char_array(buf, size);
        collision_level = obs_data.laser_obs_level;
    }
    
}


int Status_dispose::handle_sts_data(Warn_err_data & data) {
    device_sts = data.dev_sts;
    board_sts = data.ctl_sts;
    ob_3d_sts = data.ob_3d_sts;
       // std::cout<<"1111111111111111111111111111"<<" "<<ob_3d_sts.ahead_obstacle<<" "<<ob_3d_sts.rear_obstacle<<std::endl;
   //printf("11111111111111111111111111111111111 %d,%d\n",ob_3d_sts.ahead_obstacle,ob_3d_sts.rear_obstacle); 
collision_switch = data.dev_sts.collision_switch;
        if(!collision_switch) collision_level = NONE_OBS;
    if(board_sts._reset) warning_code = WARNING_NULL;
    if(device_sts.pose_initilized==false)
        warning_code = WARNING_LOCATE_INCOMPLETE;/////机器人定位未完成
    else if((collision_level==WARN_OBS_1||collision_level==WARN_OBS_2)){
        if(use_radar_type == 0)
         warning_code = WARNING_LASER_OBS_INVOKE;/////前方检测到障碍为1
        else if(use_radar_type == 1){
            if(ob_3d_sts.ahead_obstacle==warning||ob_3d_sts.ahead_obstacle==slow_down)
                warning_code = WARNING_LASER_OBS_INVOKE;//前方检测到障碍物
            else if(ob_3d_sts.rear_obstacle==warning||ob_3d_sts.rear_obstacle==slow_down)
                warning_code = WARNING_FB_OBS_INVOKE;//后方检测到障碍物
	        //cout<<"7777777777777777777777777777"<<ob_3d_sts.ahead_obstacle<<" "<<ob_3d_sts.rear_obstacle<<" "<<warning_code<<std::endl;
        }
    }
    else if(device_sts.device2_disconnect||device_sts.device1_disconnect)
        warning_code = WARNING_TWO_DIM_DISCONNECT;/////二维码传感器未连接
    else if(device_sts.imu_disconnect)
        warning_code = WARNING_IMU_DISCONNECT;/////传感器设备未连接
    else if(device_sts.laser_disconnect)
        warning_code = WARNING_LASER_DISCONNECT;/////传感器设备未连接
    else if(device_sts.board_disconnect)
        warning_code = WARNING_BOARD_DISCONNECT;/////传感器设备未连接
    //else if(board_sts._battary<warning_bat)
        //warning_code = WARNING_LOWPOWER;/////电量低为
    else
        warning_code = WARNING_NULL;
    
    if(board_sts._reset) error_code = ERROR_NULL;
    if((error_code == ERROR_NULL)||(error_code == ERROR_MOTORERROR)||(error_code == ERROR_LASER_OBS)||
    (error_code == ERROR_BACK_OBS_INVOKE)||(error_code == ERROR_LEFT_OBS_INVOKE)||(error_code == ERROR_RIGHT_OBS_INVOKE))
    {
    if(board_sts._emgcy)
        error_code = ERROR_EMERGE;
    else if(board_sts.anti_colli_f||board_sts.anti_colli_b||board_sts.anti_colli_l||board_sts.anti_colli_r)
        error_code = ERROR_ANTICRUSH;
    else if(device_sts.motor_disconnect||device_sts.motor_error)
        error_code = ERROR_MOTORERROR;
    else if(collision_level>=EMC_OBS)
    {    
        error_code = ERROR_LASER_OBS;//报error的话，默认是雷达前面进入最后一个区域内
        if(board_sts.obs_avoid_b)
        error_code = ERROR_BACK_OBS_INVOKE;/////后避障传感器触发为2
        else if(board_sts.obs_avoid_l)
        error_code = ERROR_LEFT_OBS_INVOKE;/////左避障传感器触发为3
        else if(board_sts.obs_avoid_r) 
        error_code = ERROR_RIGHT_OBS_INVOKE;/////右避障传感器触发为3
        else if(ob_3d_sts.left_obstacle == emergency_braking)
        error_code = ERROR_LEFT_OBS_INVOKE;
        else if(ob_3d_sts.right_obstacle == emergency_braking)
        error_code = ERROR_RIGHT_OBS_INVOKE;
        else if(ob_3d_sts.rear_obstacle == emergency_braking)
        error_code = ERROR_BACK_OBS_INVOKE;
    }
    else if(device_sts.dm_code_error)
        error_code = ERROR_DM_CODE_READ;/////二维码读数报错
    else
        error_code = ERROR_NULL;
    }

    std::vector<char> sendbuf;
    Warn_err_data data_out(warning_code,error_code);
    data_out.ctl_sts = board_sts;
    data_out.dev_sts = device_sts;
    data_out.ob_3d_sts = ob_3d_sts;
    data_out.dev_sts.collision_level = collision_level;
    data_out.to_char_array(&sendbuf);
    queue->send(CHANNEL_WARN_ERR, sendbuf.data(), sendbuf.size());
	return 0;
};

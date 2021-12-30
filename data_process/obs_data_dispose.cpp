#include <map>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include "../common/data_transform.h"
#include "../common/configuration.h"
#include "obs_data_dispose.h"
#include "../logic/agv.h"
#include "ridar_to_obavoid.h"
Obs_dispose::Obs_dispose(MsgQueue *queue,  int war_err_chan, int radar_chan ,int radar_3d_sts_chan)
{
    war_err_c = war_err_chan;
    radar_c = radar_chan;
    radar_3d_sts_c = radar_3d_sts_chan;
    this->queue = queue;
    queue->add_listener(war_err_c, this);
    queue->add_listener(radar_c, this);
    queue->add_listener(radar_3d_sts_c, this);
    l1x=get_configs()->get_float("radar", "collision_distance_level1x", nullptr);
    l1y=get_configs()->get_float("radar", "collision_distance_level1y", nullptr);
    l2x=get_configs()->get_float("radar", "collision_distance_level2x", nullptr);
    l2y=get_configs()->get_float("radar", "collision_distance_level2y", nullptr);
    l3x=get_configs()->get_float("radar", "collision_distance_level3x", nullptr);
    l3y=get_configs()->get_float("radar", "collision_distance_level3y", nullptr);
    thsh = get_configs()->get_int("radar", "collision_threshold", nullptr);
    use_radar_type = get_configs()->get_int("radar", "use_radar_type", nullptr);

};

Obs_dispose::~Obs_dispose() {
    queue->remove_listener(war_err_c, this);
    queue->remove_listener(radar_c, this);
    queue->remove_listener(radar_3d_sts_c, this);
};



void Obs_dispose::recieve_message(const int channel, char *buf, const int size) {
    if (channel == war_err_c)
    {
        Warn_err_data err_data;
        err_data.from_char_array(buf, size);
		handle_sts_data(err_data);
    }else if (channel == radar_c && use_radar_type == 0)
    {

        PointCloudData laser_data(0);
        laser_data.from_char_array(buf, size);
        radar_pos = get_global_agv_instance()->expolate_current_position_tim(laser_data.timestamp);
        radar_raw_pos = get_global_agv_instance()->expolate_current_position_tim_raw(laser_data.timestamp);
        handle_radar_data(laser_data);
        last_radar_data = laser_data;
        _recor_data2map();
        radar_cnt++;
        if(radar_cnt>100) radar_cnt = 0;
	}
    else if(channel == radar_3d_sts_c && use_radar_type == 1){
        Laser_obs_3d_data ob_3d_sts;
        ob_3d_sts.from_char_array(buf,size);
        handle_radar_3d_data(ob_3d_sts);

    }
    
}
void Obs_dispose::begin_high_resolution_map_create()
{
    
    map_raw_data_file = fopen("map.rawmap","w+");
    usleep(1000000);
	record_order = 1;
    first_record = true;
    begin_mapping = true;
}
void Obs_dispose::end_high_resolution_map_create()
{
    begin_mapping = false;
	usleep(1000000);
    fclose(map_raw_data_file);

}
void Obs_dispose::_recor_data2map()
{ 
	if(!begin_mapping) return;
	double xp,yp,laser_len;
	if (fabs(last_record_pos.x - radar_pos.x)>0.05||fabs(last_record_pos.y - radar_pos.y)>0.05||first_record||radar_cnt>5)//||fabs(last_record_pos.theta - radar_pos.theta)>1.0
	{
        if(radar_cnt>5)radar_cnt = 0;
	//fprintf(map_raw_data_file,"%d\t",record_order);
	if(first_record) first_record = false;

	///////////小梦要求格式输出地图创建所需数据///////////////////////////
    //fprintf(map_raw_data_file,"%f\t%f\t%f\t%d\t", radar_pos.x,radar_pos.y,radar_pos.theta,last_radar_data.points.size());
    fprintf(map_raw_data_file,"%f %f %f ,%f %f %f ,", radar_pos.x,radar_pos.y,radar_pos.theta,radar_raw_pos.x,radar_raw_pos.y,radar_raw_pos.theta);
    
    for(int i=0;i<last_radar_data.points.size();i++)
    {
        
    xp = last_radar_data.points[i](0);
    yp = last_radar_data.points[i](1);
    laser_len = last_radar_data.intensities[i];
    fprintf(map_raw_data_file,"%f %f %f ,",xp,yp,laser_len);
    }
    fprintf(map_raw_data_file,"\n");   	
	////////////////////////////////////////////////////////////////////
	record_order++;
	last_record_pos = radar_pos;
	}
}

int Obs_dispose::handle_radar_data(PointCloudData & data) {
    collision_switch = device_sts.collision_switch;
    colli_cnt1 = 0;
	colli_cnt2 = 0;
	colli_cnt3 = 0;
    bool is_invoke;
    int size = data.points.size();
    for (int i = 0; i<size; i++)
    {
        double xp = data.points[size-i-1](0);
        double yp = data.points[size-i-1](1);
        xp -= 0.1;
        is_invoke = false;
        if(xp>0)
        {
            if((xp>0.0)&&(xp<l1x)&&(yp<l1y)&&(yp>0-l1y)){
            colli_cnt1++;
            is_invoke = true;

            }
            if((xp>0.0)&&(xp<l2x)&&(yp<l2y)&&(yp>0-l2y)){
            colli_cnt2++;
            is_invoke = true;

            }
            if((xp>0.0)&&(xp<l3x)&&(yp<l3y)&&(yp>0-l3y)){
            colli_cnt3++;
            is_invoke = true;

            }
        }

        if (colli_cnt1>thsh) break;
        //{printf("colli_cnt1=%d break\n",colli_cnt1);break;}
        //if (colli_cnt2>thsh) break;
        //{printf("colli_cnt2=%d break\n",colli_cnt2);break;}
        //if (colli_cnt3>thsh) break;
        //{printf("colli_cnt3=%d break\n",colli_cnt3);break;}

        /*if(!is_invoke){
        if(colli_cnt1>2)colli_cnt1--;////仅当有连续障碍满足条件时才触发避障
        if(colli_cnt2>2)colli_cnt2--;
        if(colli_cnt3>2)colli_cnt3--;
        }*/
    }
    int new_level = NONE_OBS;
    
    if(colli_cnt1>thsh){
        new_level=EMC_OBS;
    }else if(colli_cnt2>thsh){
        new_level=WARN_OBS_2;
    }else if(colli_cnt3>thsh){
        new_level=WARN_OBS_1;
    }else{
        new_level = NONE_OBS;
    }
    if(!collision_switch) new_level = NONE_OBS;
    if(laser_collision_level!=new_level){
        laser_collision_level=new_level;
    }


    return 0;
};

int Obs_dispose::handle_sts_data(Warn_err_data & errdata) {
    device_sts = errdata.dev_sts;
    board_sts = errdata.ctl_sts;
    collision_switch = device_sts.collision_switch;

    int new_level = NONE_OBS;

    if(board_sts.obs_avoid_f||board_sts.obs_avoid_b)
    //new_level = WARN_OBS_2;/////前后避障传感器触发为2
    new_level = EMC_OBS_P;/////前后避障传感器触发为2
    if(board_sts.obs_avoid_l||board_sts.obs_avoid_r)
    //new_level = WARN_OBS_2;/////左右避障传感器触发为2
    new_level = EMC_OBS_P;/////左右避障传感器触发为2
    //if(board_sts.anti_colli_f||board_sts.anti_colli_f)
    //new_level = EMC_OBS;/////前后防撞传感器触发为2



    if(!collision_switch) new_level = NONE_OBS;
    if(point_laser_collision_level!=new_level){
        point_laser_collision_level=new_level;
    } 
    //printf ("board_sts.obs_avoid_f=%d,board_sts.obs_avoid_b=%d\n",board_sts.obs_avoid_f,board_sts.obs_avoid_b);
    //printf ("board_sts.obs_avoid_l=%d,board_sts.obs_avoid_r=%d\n",board_sts.obs_avoid_l,board_sts.obs_avoid_r);
    //printf ("point_laser_collision_level=%d,laser_collision_level=%d\n",point_laser_collision_level,laser_collision_level);
    //这里的避障等级是融合的点激光和激光雷达之后的避障等级
    collision_level = ((laser_collision_level)>=(point_laser_collision_level)?(laser_collision_level):(point_laser_collision_level));
    //printf ("point_laser_collision_level=%d,laser_collision_level=%d，collision_level=%d\n",point_laser_collision_level,laser_collision_level,collision_level);
    
    if(use_radar_type == 0){
        std::vector<char> sendbuf;
        Laser_obs_data out_data(0);
        out_data.laser_obs_level = collision_level;
        out_data.laser_obs_level_l = laser_collision_level;
        out_data.laser_obs_level_p = point_laser_collision_level;
        out_data.to_char_array(&sendbuf);
        queue->send(CHANNEL_LASER_OBS_OUT, sendbuf.data(), sendbuf.size());
    }
    else{
        std::vector<char> sendbuf;
        Laser_obs_3d_data out_data;
        out_data.laser_obs_level = collision_level;
        out_data.laser_obs_level_l = laser_collision_level;
        out_data.laser_obs_level_p = point_laser_collision_level;
        out_data.laser_obs_level_f_l = radar_3d_sts.ahead_obstacle;
        out_data.laser_obs_level_b_l = radar_3d_sts.rear_obstacle;
        out_data.laser_obs_level_l_l = radar_3d_sts.left_obstacle;
        out_data.laser_obs_level_r_l = radar_3d_sts.right_obstacle;
        out_data.to_char_array(&sendbuf);
        queue->send(CHANNEL_LASER_OBS_OUT, sendbuf.data(), sendbuf.size());
    }
	return 0;
};
int Obs_dispose::handle_radar_3d_data(Laser_obs_3d_data& ob_data){
    collision_switch = device_sts.collision_switch;
    radar_3d_sts.ahead_obstacle = ob_data.laser_obs_level_f_l;
    radar_3d_sts.rear_obstacle = ob_data.laser_obs_level_b_l;
    radar_3d_sts.left_obstacle = ob_data.laser_obs_level_l_l;
    radar_3d_sts.right_obstacle = ob_data.laser_obs_level_r_l;
    int new_level = NONE_OBS;
    int ob_level = std::min(ob_data.laser_obs_level_f_l,std::min(ob_data.laser_obs_level_b_l,std::min(ob_data.laser_obs_level_l_l,ob_data.laser_obs_level_r_l)));
    if(ob_level == normal_run){
        new_level = NONE_OBS;
    }
    else if(ob_level == warning){
        new_level=WARN_OBS_1;
    }
    else if(ob_level == slow_down){
        new_level=WARN_OBS_2;
    }
    else if(ob_level == emergency_braking){
        new_level=EMC_OBS; 
    }
    else
        new_level = NONE_OBS;
    if(!collision_switch) new_level = NONE_OBS;
    if(laser_collision_level!=new_level){
        laser_collision_level=new_level;
    }

};
#include <stdio.h>
#include <cmath>
//#include "../logic/agv.h"
#include "odometer.h"
#include "../common/configuration.h"

FILE *fp_angle_imu;
FILE *fp_odom_w;

Odometer::Odometer(MsgQueue *queue, int imu_chan, int wheel_chan, int odom_chan) {
    imu_c = imu_chan;
    wheel_c = wheel_chan;
    odom_c = odom_chan;
    this->queue = queue;
    robot_control_type = get_configs()->get_int("odom", "robot_control_model", nullptr);
    if(robot_control_type<ROBOT_DIFF_MODEL_CARRY) robot_control_type = ROBOT_DIFF_MODEL_CARRY;////默认采用差分模型

    pthread_mutex_init(&list_mutex, nullptr);
	left_ready = false;
	right_ready = false;
	first_read = true;
    queue->add_listener(imu_c, this);
    queue->add_listener(wheel_c, this);
	fp_angle_imu = fopen("angle_odo_imu.txt","w");
	fp_odom_w = fopen("odom11.txt","w");


}

Odometer::~Odometer() {
    queue->remove_listener(imu_c, this);
    queue->remove_listener(odom_c, this);
    pthread_mutex_destroy(&list_mutex);
}

int Odometer::handle_wheel_data(WheelSensorData & data) {
    //fprintf(fp_odom_w,"Odom: %ld, %d, %ld\n", data.timestamp, data.type, data.data);
    int type = data.type;
    bool do_update = true;
    double d_theta, k,d_vel;
	long time_pose_cur;
    bool new_data_recved = true;
    pthread_mutex_lock(&list_mutex);
    if(type == WHEEL_DATA_TYPE_LEFT) {
	    if(left_datas.size() > 2)
        if ((data.timestamp - left_datas.back().timestamp)<0)
        new_data_recved = false;
        if (new_data_recved)
        left_datas.push_back(data);
        if(left_datas.size() > 2) {
            left_datas.pop_front();
        }
    }
    if(type == WHEEL_DATA_TYPE_RIGHT) {
        if(right_datas.size() > 2)
        if ((data.timestamp - right_datas.back().timestamp)<0)
        new_data_recved = false;
        if (new_data_recved)
        right_datas.push_back(data);
        if(right_datas.size() > 2) {
            right_datas.pop_front();
        }
    }
//////////////////the follow section is excute only the dual steer wheel is option
    if(type == WHEEL_DATA_TYPE_STEER_FV1) {
        if(front_l_datas.size() > 2)
        if ((data.timestamp - front_l_datas.back().timestamp)<0)
        new_data_recved = false;
        if (new_data_recved)
        front_l_datas.push_back(data);
        if(front_l_datas.size() > 2) {
            front_l_datas.pop_front();
        }
    }
    if(type == WHEEL_DATA_TYPE_STEER_FR1) {
        if(front_r_datas.size() > 2)
        if ((data.timestamp - front_r_datas.back().timestamp)<0)
        new_data_recved = false;
        if (new_data_recved)        
        front_r_datas.push_back(data);
        if(front_r_datas.size() > 2) {
            front_r_datas.pop_front();
        }
    }
    if(type == WHEEL_DATA_TYPE_STEER_BV1) {
        if(back_l_datas.size() > 2)
        if ((data.timestamp - back_l_datas.back().timestamp)<0)
        new_data_recved = false;
        if (new_data_recved)           
        back_l_datas.push_back(data);
        if(back_l_datas.size() > 2) {
            back_l_datas.pop_front();
        }
    }
    if(type == WHEEL_DATA_TYPE_STEER_BR1) {
        if(back_r_datas.size() > 2)
        if ((data.timestamp - back_r_datas.back().timestamp)<0)
        new_data_recved = false;
        if (new_data_recved)  
        back_r_datas.push_back(data);
        if(back_r_datas.size() > 2) {
            back_r_datas.pop_front();
        }
    }
////////////////////////////////////////////////

    if(first_read)
    {
    last_t  =data.timestamp;      
    if(type == WHEEL_DATA_TYPE_LEFT){last_left = data.data;left_ready = true;}
    if(type == WHEEL_DATA_TYPE_RIGHT){last_right =data.data;right_ready = true;}

    if(type == WHEEL_DATA_TYPE_STEER_FV1) {last_fl1 = data.data;fl1_ready = true;}
    if(type == WHEEL_DATA_TYPE_STEER_FR1) {last_fr1 = data.data;fr1_ready = true;}
    if(type == WHEEL_DATA_TYPE_STEER_BV1) {last_bl1 = data.data;bl1_ready = true;}
    if(type == WHEEL_DATA_TYPE_STEER_BR1) {last_br1 = data.data;br1_ready = true;}

    if(robot_control_type == ROBOT_DIFF_MODEL_CARRY||(robot_control_type == ROBOT_DIFF_MODEL_COMMON)||(robot_control_type == ROBOT_DIFF_MODEL_COMMON_YB))///差分模型
    {
    if((left_ready)&&(right_ready)&&imu_ready)
    {first_read = false;left_ready = false;right_ready=false;}
    else do_update = false;
    }
    else if(robot_control_type == ROBOT_DUAL_STEER_MODEL)///双舵轮模型
    {
    if((fl1_ready)&&(fr1_ready)&&bl1_ready&&br1_ready&&imu_ready)
    {first_read = false;fl1_ready = false;fr1_ready=false;bl1_ready=false;br1_ready=false;}
    else do_update = false;        
    }
    }


    if(robot_control_type == ROBOT_DIFF_MODEL_CARRY||(robot_control_type == ROBOT_DIFF_MODEL_COMMON)||(robot_control_type == ROBOT_DIFF_MODEL_COMMON_YB))///差分模型
	{
    if(left_datas.size() < 2 ||right_datas.size() < 2 ||imu_datas.size() < 2 ) 
    {    
        pthread_mutex_unlock(&list_mutex);
        printf("List size : left %d, right %d, imu %d\n", left_datas.size(), right_datas.size(), imu_datas.size()); 
        return 0;
    } 
    }
    else if(robot_control_type == ROBOT_DUAL_STEER_MODEL)///双舵轮模型
    {
    if(front_l_datas.size() < 2 ||front_r_datas.size() < 2 
    ||back_l_datas.size() < 2 ||back_r_datas.size() < 2 
    ||imu_datas.size() < 2 ) 
    {    
        pthread_mutex_unlock(&list_mutex);
        printf("List size : front_l %d, front_r %d,back_l %d, back_r %d, imu %d\n", front_l_datas.size(), front_r_datas.size(),
        back_l_datas.size(), back_r_datas.size(), imu_datas.size()); 
        return 0;
    } 

    }	
    if(!new_data_recved) return -1;
    {
        count = 0;
        if(type == WHEEL_DATA_TYPE_LEFT) {
            cur_t = left_datas.back().timestamp;
            cur_left = left_datas.back().data;
            do_update = true;
            left_ready = true;
        }
        if(type == WHEEL_DATA_TYPE_RIGHT)
        {
            cur_t = right_datas.back().timestamp;
            cur_right = right_datas.back().data;
            do_update = true;
            right_ready = true;
        }

        if(type == WHEEL_DATA_TYPE_STEER_FV1)
        {
            cur_t = front_l_datas.back().timestamp;
            cur_fl1 = front_l_datas.back().data;
            do_update = true;
            fl1_ready = true;
        }		
        if(type == WHEEL_DATA_TYPE_STEER_FR1)
        {
            cur_t = front_r_datas.back().timestamp;
            cur_fr1 = front_r_datas.back().data;
            do_update = true;
            fr1_ready = true;
        }
        if(type == WHEEL_DATA_TYPE_STEER_BV1)
        {
            cur_t = back_l_datas.back().timestamp;
            cur_bl1 = back_l_datas.back().data;
            do_update = true;
            bl1_ready = true;
        }		
        if(type == WHEEL_DATA_TYPE_STEER_BR1)
        {
            cur_t = back_r_datas.back().timestamp;
            cur_br1 = back_r_datas.back().data;
            do_update = true;
            br1_ready = true;
        }		
        k = imu_datas.back().angular_velocity_z - imu_datas.front().angular_velocity_z;
        k = k / (imu_datas.back().timestamp_us - imu_datas.front().timestamp_us);
        double w1 = imu_datas.back().angular_velocity_z;
        double w2 = w1;
        w1 += (last_t - imu_datas.back().timestamp_us) * k;
        w2 += (cur_t - imu_datas.back().timestamp_us) * k;
        d_theta = ((w1 + w2) * (cur_t - last_t)) / 2.;
        d_theta = d_theta / 1000000.;
        cur_imua = imu_datas.back().angle_z;
        d_theta = -(cur_imua - last_imua);
        /////////////////////////////////对线加速度进行插补d_vel
        k = imu_datas.back().linear_acceleration_x - imu_datas.front().linear_acceleration_x;
        k = k / (imu_datas.back().timestamp_us - imu_datas.front().timestamp_us);
        double acc1 = imu_datas.back().linear_acceleration_x;
        double acc2 = acc1;
        acc1 += (last_t - imu_datas.back().timestamp_us) * k;
        acc2 += (cur_t - imu_datas.back().timestamp_us) * k;
        d_vel = ((acc1 + acc2) * (cur_t - last_t)) / 2.;
        d_vel = d_vel / 1000000.;
    }
    pthread_mutex_unlock(&list_mutex);
	
    if(robot_control_type == ROBOT_DIFF_MODEL_CARRY||(robot_control_type == ROBOT_DIFF_MODEL_COMMON)||(robot_control_type == ROBOT_DIFF_MODEL_COMMON_YB))///差分模型
	{if((left_ready == false)||(right_ready == false)) do_update = false;}
    else if(robot_control_type == ROBOT_DUAL_STEER_MODEL)///双舵轮模型
    {
        if((fl1_ready == false)||(fr1_ready == false)||(bl1_ready == false)||(br1_ready == false)) do_update = false;
    }
    if(!do_update) return 0;
        //printf("Skip odometer data %ld: Not ready\n", data.timestamp);
        
    //cur_t = get_current_time_us();
    Position pose(cur_t, 0, 0, 0);
    ImuData2D imudata(cur_t, 0, 0, 0);
    std::vector<char> tmp2;
	int ret ;
    if(robot_control_type == ROBOT_DIFF_MODEL_CARRY||(robot_control_type == ROBOT_DIFF_MODEL_COMMON)||(robot_control_type == ROBOT_DIFF_MODEL_COMMON_YB))///差分模型
    {
        ret = my_model.forward_kine(&pose,&imudata, cur_t - last_t, cur_left- last_left, cur_right- last_right, d_theta,d_vel);
        if(ret == 0) {
        // printf("Odometer : Change current pose (%.3f, %.3f, %.3f)\n", pose.x, pose.y, pose.theta);
        std::vector<char> tmp;
        pose.to_char_array(&tmp);
        queue->send(odom_c, tmp.data(), tmp.size());

        //imudata.to_char_array(&tmp2);
        //queue->send(CHANNEL_ODOM_IMU, tmp2.data(), tmp2.size());
        //if(fabs(imu_datas.back().linear_acceleration_x)>0.1)
        //{
        //printf("imu acc_x=%f,acc_y=%f,ang_vel=%f\n",imu_datas.back().linear_acceleration_x,imu_datas.back().linear_acceleration_y,imu_datas.back().angular_velocity_z);
        //printf("odom acc_x=%f,acc_y=%f,ang_vel=%f\n",imudata.linear_acceleration_x,imudata.linear_acceleration_y,imudata.angular_velocity_z);
        //}
        }
        last_left = cur_left;
        last_right = cur_right;
        left_ready = false;
        right_ready = false;
        last_t = cur_t;
    }
    else if(robot_control_type == ROBOT_DUAL_STEER_MODEL)///双舵轮模型
    {
        ret = my_model.forward_kine(&pose,&imudata, cur_t - last_t,cur_fl1-last_fl1, cur_fr1,cur_bl1-last_bl1, cur_br1, d_theta,d_vel);
        if(ret == 0) 
        {
        // printf("Odometer : Change current pose (%.3f, %.3f, %.3f)\n", pose.x, pose.y, pose.theta);
        //printf("Odometer : fr br (%.3f, %.3f)\n",  pose.rot_rf1, pose.rot_rb1);
        std::vector<char> tmp;
        pose.to_char_array(&tmp);
        queue->send(odom_c, tmp.data(), tmp.size());

        //imudata.to_char_array(&tmp2);
        //queue->send(CHANNEL_ODOM_IMU, tmp2.data(), tmp2.size());
        }
        last_fl1 = cur_fl1;
        last_fr1 = cur_fr1;
        last_bl1 = cur_bl1;
        last_br1 = cur_br1;
        fl1_ready = false;
        fr1_ready = false;
        bl1_ready = false;
        br1_ready = false;
        last_t = cur_t;
    }
    last_imua = cur_imua;
    return ret;
}
double Odometer::obtain_imu_angle()
{
	double ret_ang = imu_datas.back().angle_z-start_mapping_angle;
	return ret_ang;
}

void Odometer::recieve_message(const int channel, char *buf, const int size) {
    if(channel == imu_c) {
        ImuData2D data;
        data.from_char_array(buf, size);
        handle_imu_data(data);
        //printf("odom insert imu data %ld\n", data.timestamp_us);
    }
    else if(channel == wheel_c) {
        WheelSensorData data;
        data.from_char_array(buf, size);
        handle_wheel_data(data);
        //printf("odom insert wheel data %ld %d\n", data.timestamp, data.type);
    }
}


long Odometer::linear_interpolation(std::list<WheelSensorData> *datas, long time) {

    double k = datas->back().data - datas->front().data;
    k = k / (datas->back().timestamp - datas->front().timestamp);
    long d = (long)(k * (time - datas->back().timestamp));
    return d + datas->back().data;
}



int Odometer::handle_imu_data(ImuData2D & data) {
    
    // printf("IMU: %ld, %.3f\n", data.timestamp_us, data.angular_velocity_z);
    pthread_mutex_lock(&list_mutex);
    if(imu_datas.size() > 2)
    {
        if ((data.timestamp_us - imu_datas.back().timestamp_us)<0)
        {
            pthread_mutex_unlock(&list_mutex);
            printf("imu timestamp is too small  exit\n");
            return -1;
        }
    }
    imu_datas.push_back(data);
    
	if(first_read)
	{
	//printf("imu data received\n");
	last_imua = data.angle_z;imu_ready = true;
	//printf("imu data initilized \n");
	}
	if(start_mapping)
	{
	start_mapping = false;
	start_mapping_angle = data.angle_z;
	printf("imu data start_mapping_angle = %f\n");
	}
    if(imu_datas.size() > 2) {
        imu_datas.pop_front();
    }
    pthread_mutex_unlock(&list_mutex);

    return 0;
}

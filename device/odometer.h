#ifndef AGV_ODOMETER_MODULE_H
#define AGV_ODOMETER_MODULE_H

#include <list>

#include "../common/pose.h"
#include "../common/sensor_data.h"
#include "../common/msg_queue.h"
#include "../data_out_process/walking_machine.h"

class Odometer : public MsgQueueListener {
public:
    Odometer(MsgQueue *queue, int imu_chan, int wheel_chan, int odom_chan);
    ~Odometer();

    int handle_wheel_data(WheelSensorData & data);
    int handle_imu_data(ImuData2D & data);
    
    virtual void recieve_message(const int channel, char *buf, const int size);
	bool start_mapping;
	double obtain_imu_angle();
    
private:
    std::list<WheelSensorData> left_datas;
    std::list<WheelSensorData> right_datas;
    std::list<WheelSensorData> front_l_datas;
    std::list<WheelSensorData> front_r_datas;
    std::list<WheelSensorData> back_l_datas;
    std::list<WheelSensorData> back_r_datas;
    std::list<ImuData2D> imu_datas;
    pthread_mutex_t list_mutex;

    int skip = 0;
    int count = 0;

    MsgQueue *queue;
    int imu_c;
    int wheel_c;
    int odom_c;
    bool first_read;
	long last_left, last_right, last_t;
    long last_fl1, last_fr1;////上一步前舵轮位移1，角度1
    long last_bl1, last_br1;////上一步后舵轮位移1，角度1
	long cur_t, cur_left, cur_right;
    long cur_fl1, cur_fr1;////当前前舵轮位移1，角度1
    long cur_bl1, cur_br1;////当前后舵轮位移1，角度1
	bool left_ready,right_ready,imu_ready;
    bool fl1_ready,fr1_ready,bl1_ready,br1_ready;
	double cur_imua,last_imua;
	double start_mapping_angle;
	int robot_control_type = 0;
    long linear_interpolation(std::list<WheelSensorData> *datas, long time);
    Model_kine my_model;

};


#endif


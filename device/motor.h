#ifndef MOTOR_H_
#define MOTOR_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
#include <vector>
#include "../data_out_process/lift_machine.h"
#include "../common/struct_drive.h"
#include "chassis_diff.h"
#include "dual_steer.h"
#include "../common/sensor_data.h"
#include "../common/time.h"
#include "../common/msg_queue.h"
#include "../common/move.h"
#include "../data_out_process/data_out.h"

class Motor : public MsgQueueListener
{
public:
    Motor(MsgQueue *queue, int motor_chan, int wheel_chan, int move_chan);
    ~Motor();
    void MotorDataAnalyze(MotorTypedef *thisMotor, u8 *data);
    void MotorRun(Data_out data);
    void MotorRun(float vl,float vr);
    void SteerWheel_MotorRun(Data_out data);
    void SteerWheel_MotorRun(double front_Velocity,double back_Velocity,double front_Angular,double back_Angular);
 
    void MotorStop(void);
    void motorPositionControl(MotorTypedef thisMotor);
    void motorSpeedControl(MotorTypedef thisMotor);
    void motorSetPositionMode(MotorTypedef thisMotor);
    void motorSetSpeedMode(MotorTypedef thisMotor); 
    void SwitchPDO(MotorTypedef thisMotor);
    void handle_motor_data(Data_out data);
    void handle_steer_motor_data(Data_out data);
	void handle_lift_data(Data_out data);
    virtual void recieve_message(const int channel,char *buf, const int size);

    bool reset_motor();
    void motorDisable(MotorTypedef thisMotor);

    int init_can();
    void end_can(int sock);
    int can_send(int sock,canid_t id,unsigned char* data);
    int can_receive(int sock,can_frame *fr2);
    static void* monitor_motor_state(void* param);
    int detect_cnt = 0;
    bool motor_error = false;
    bool motor_disconnect = true;
    Lifter lft_drv;////
    Dual_steer dual_steer_drv;
    Chassis_drive  diff_drv;
    void handle_music_data(Data_out data);
    void play_music(u8 can_id,u8 index,u8 volume);
    void handle_out_data(Data_out data);
    void send_io_ctl(u8 can_id,st_led out);
    u8 CalculateChecksum(u8 *aubData_p, int auwDataLength);
    int io_board_can_enable;

private:
    /**
 * @brief Motor Externed
 */
    void MotorDataAnalyze_YB(MotorTypedef *thisMotor, u8 *data,char an_type);
    void motorSpeedControl_YB(MotorTypedef thisMotor);
    void motorSetSpeedMode_YB(MotorTypedef thisMotor); 
    void motorDisable_YB(MotorTypedef thisMotor);
    void motorEnable_YB(MotorTypedef thisMotor);
    MsgQueue *queue;
    int wheel_c;
    int motor_c;
    int move_c;
    int level;
    u8 MotorError_Flag;
    int Model_type;
    pthread_t recieve_wheel_thread;
    pthread_t monitor_motor_thread;
    pthread_t send_wheel_thread;
    pthread_mutex_t recieve_2wheel;
    pthread_mutex_t monitor_motor;
    static void* recieve_wheel(void* param);
    u8 *wheel_data;
    can_frame fr_recieve;
    bool send_thread_running =true;
    bool recieve_thread_running =true;
    int s;
    double velocity;
    double angular;
    bool is_arrive;
    int dir = -1;//0为下降，1为上升，-1为待命
    int voice_volum;
};

#endif /* MOTOR_H_ */

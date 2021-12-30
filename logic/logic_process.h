#ifndef LOGIC_PROCESS_H_
#define LOGIC_PROCESS_H_

#include <vector>
#include "../common/sensor_data.h"
#include "../common/time.h"
#include "../common/warn_err_data.h"
#include "../common/speed_data.h"
#include "../common/msg_queue.h"
#include "../common/move.h"
#include "../device/control_board.h"
#include "../data_out_process/data_out.h"
#include "../data_out_process/walking_machine.h"
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
enum {
    NONE, LOCALCONTROL, BLUETOOTHCONTROL, CLOUDCONTROL, HANDLECONTROL
} control;/*!< 控制权,enum {NONE,LOCALCONTROL,BLUETOOTHCONTROL,CLOUDCONTROL,HANDLECONTROL} */

enum {
    NORMALSTATE, WARNINGSTATE, ERRORSTATE
} health;/*!< 小车健康状态,enum {NORMALSTATE, WARNINGSTATE, ERRORSTATE} */

typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned int u32;
class Logic_process : public MsgQueueListener
{
public:

    Logic_process(MsgQueue *queue,  int war_err_chan, int speed_chan, int lift_chan);
    ~Logic_process();

    virtual void recieve_message(const int channel,char *buf, const int size);
    static void* output_disposed_data(void* param);
    void error_code_dispose();
    void warning_code_dispose();
    void normal_state_run();/////机器人正常运行
    void warnning_state_run();/////机器人报警运行
    void error_state_run();/////机器人错误运行
    st_board_sts board_sts;
    st_device_sts device_sts;
    obavoid_state_3d ob_3d_sts;

    Laser_obs_3d_data obs_3d_data;
    Laser_obs_data obs_data;
    
    st_led  led_in;
    st_motion  move_in;
    st_lift  lift_in;
    st_voice  voice_in;
    Data_out logic_out;
    char agv_cur_state;
    char f_led_voice_recv;
    char f_move_recv;
    char f_lift_recv;
    float speed_ratio;
    Speed_data speed_data;
    Model_kine *kine_slove = nullptr;
    //Model_kine kine_slove;
    int lift_up_cnt = 0;
    int lift_down_cnt = 0;
private:
	/**
 * @brief Motor Externed
 */
    MsgQueue *queue;
    int war_err_c;
    int speed_c;
    int lift_c;
    int output_c;
    pthread_t output_thd;
    pthread_mutex_t mutex;
    int detect_cnt = 0;
    char err_code;
    char warn_code;
    double vel_line;
    double vel_angle;
    double rot_rf;
    double rot_rb;
    char last_led;
    char last_voice;
    bool up_flag = false;
    bool down_flag = false;
    int use_radar_type;
};
#endif /* MOTOR_H_ */

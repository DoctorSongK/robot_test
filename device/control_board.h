#ifndef CONTROL_BOARD_H_
#define CONTROL_BOARD_H_

#include <vector>
#include "../common/msg_queue.h"
#include "../data_out_process/broadcast.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
enum {
    MANUAL_MODE, AUTO_MODE
} control_state;/*!< 小车控制方式方式 */
typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned int u32;

typedef struct {

	u32 Input; /*!< Input */

	u8 GreenButton; /*!< 绿色按钮 */
	u8 BlueButton; /*!< 蓝色按钮 */
	u8 EmergeButton; /*!< 急停自锁按钮 */
	u8 OrangeButton; /*!< 橙色自锁按钮 */

	u8 FrontAntiCrush; /*!< 前 防撞条 */
	u8 RearAntiCrush; /*!< 后 防撞条 */

	u8 LeftInfrared1; /*!< 红外-左1 */
	u8 RightInfrared1; /*!< 红外-右1*/
	u8 LeftInfrared2; /*!< 红外-左2 */
	u8 RightInfrared2; /*!< 红外-右2 */	
	u8 FrontInfrared1; /*!< 红外-前1 */
	u8 FrontInfrared2; /*!< 红外-前2 */
	u8 BackInfrared1; /*!< 红外-后1*/
	u8 BackInfrared2; /*!< 红外-后2*/   //从左往右（1、2的区分）

    u8 charging_flag; /*!正在充电标识*/   //从左往右（1、2的区分）

} SignalInTypedef;

class Control_board : public MsgQueueListener
{
public:
    Control_board(MsgQueue *queue,  int _chan);
    ~Control_board();
    int init_can();
    void end_can(int hd);
    virtual void recieve_message(const int channel,char *buf, const int size);
    int can_receive(int sock,can_frame *fr2);   
    static void* recieve_io_data(void* param);
    static void* commu_detect_thread(void* param);///通信监测线程
    void reset_motor();
    void IoDataAnalyze( u8 *data);
    bool obs_avoid_f = false;//////避障传感器
    bool obs_avoid_b = false;
    bool obs_avoid_l = false;
    bool obs_avoid_r = false;

    bool anti_colli_f = false;//////防撞条
    bool anti_colli_b = false;
    bool anti_colli_l = false;
    bool anti_colli_r = false;

    bool _emgcy = false;//////急停
    bool _stop = false;//////暂停
    bool _continu = false;////////继续
    bool _reset = false;//////复位
    char _manual_auto = 0;//////手动-0 ，自动-1	
    char _battary;////电池电量
    char charging_flag;////充电标识
    bool _battary_warning = false;
    void update_agv_status();
    void update_agv_warning_error();
    void SignalInputDataAnalyze(void);
    int warning_bat;
    bool board_disconnect = false;
    int can_send(int sock,canid_t id,unsigned char* data);
    int io_board_can_enable =0;
private:
	/**
 * @brief Externed
 */
    void handle_out_data(Data_out data);
    void send_io_ctl(u8 can_id,st_led out);
    SignalInTypedef SignalIn;
    MsgQueue *queue;
    int logic_out_c;
    int detect_cnt = 0;
    pthread_t recieve_io_thd;
    pthread_t commu_detect_thd;
    int level;
    int handle_can;
    bool io_thread_running = false;
    pthread_mutex_t mutex;
};
    
#endif /* MOTOR_H_ */

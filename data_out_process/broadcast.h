#ifndef MUSIC_PLAY_H_
#define MUSIC_PLAY_H_
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include "led.h"
#include "../common/pose.h"
#include "../common/capture_point.h"
#include "../common/msg_queue.h"
#include "../data_out_process/data_out.h"
#include "../device/control_board.h"
#define LOBYTE(w) ((char)(((int)(w))&0xff))
#define HIBYTE(w) ((char)((((int)(w))>>8)&0xff))


/**
 * @}
 */

/** @defgroup voiceState_define_Constants
 * @{
 */
#define VOICE_MUSIC					1
#define VOICE_NONE					2
#define VOICE_TURNRIGHT			3
#define VOICE_CHARGENOTICE	4
#define VOICE_LACKMATERIAL	5
#define VOICE_INSIDEOBAV		6
#define VOICE_STATIONCALL		7
#define VOICE_EMERGE				8
#define VOICE_OUTLINE				9
#define VOICE_ERRORCLEAR		10
#define VOICE_LACKPOINT				11
#define VOICE_NETERROR				12
#define VOICE_ANTICRUSH				13
#define VOICE_MOTORERROR			14
#define VOICE_MATERIALDETECTWARNING		15
#define VOICE_POWERSYSTEMERROR				16
#define VOICE_ROUTELOCK								17
#define VOICE_CTL_BOARD_DISCONNECT				18
#define VOICE_TWO_DIM_DISCONNECT								19
/** @defgroup warningCode_define_Constants
 * @{
 */
#define WARNING_NULL    								0		/*!< 无告警 */
#define WARNING_OBAVINSIDE					 		1		/*!< 告警-避障停车 */
#define WARNING_LACKPOINT 							2		/*!< 告警-漏点 */
#define WARNING_MATERIALDETECTWARNING		3		/*!< 告警-物料检测故障 */
#define WARNING_LOWPOWER    						4		/*!< 告警-低电量 */
#define WARNING_SIDEOBAVINSIDE					5		/*!< 告警-侧面避障停车 */

#define WARNING_FB_OBS_INVOKE					8		/*!< 告警-激光后方避障停车 */
#define WARNING_LASER_OBS_INVOKE				9		/*!< 告警-激光前方避障停车 */
#define WARNING_LOCATE_INCOMPLETE				10		/*!< 告警-机器人定位未完成 */
#define WARNING_BOARD_DISCONNECT				21		/*!<控制板未连接 */
#define WARNING_LASER_DISCONNECT				22		/*!<激光未连接 */
#define WARNING_TWO_DIM_DISCONNECT				23		/*!<二维码未连接 */
#define WARNING_IMU_DISCONNECT				    24		/*!< IMU未连接 */

/**
 * @}
 */

/** @defgroup errorCode_define_Constants
 * @{
 */
#define ERROR_NULL                      0		/*!< 无故障 */
#define ERROR_EMERGE                    1		/*!< 故障-急停 */
#define ERROR_LASER_OBS                 2		/*!< 激光避障 */
#define ERROR_LASER_OBS_P               3		/*!< 点激光避障 */
#define ERROR_ANTICRUSH                 4		/*!< 故障-防撞条碰撞 */
#define ERROR_MOTORERROR                5		/*!< 故障-电机故障 */
#define ERROR_PAUSE                     6		/*!< 故障-暂停 */

#define ERROR_LEFT_OBS_INVOKE           18		/*!< 故障-左侧障碍触发 */
#define ERROR_RIGHT_OBS_INVOKE          19		/*!< 故障-右侧障碍触发 */
#define ERROR_BACK_OBS_INVOKE           20		/*!< 故障-后避障停车 */
#define ERROR_DM_CODE_READ              21		/*!< 二维码读数报错 */
typedef unsigned char u8;

class Io_music_modual : public MsgQueueListener {
public:
    Io_music_modual(MsgQueue *queue,  int data_iput);
    ~Io_music_modual();
    virtual void recieve_message(const int channel, char *buf, const int size);
	
    MsgQueue *  queue;

    void play_music(u8 can_id,u8 index,u8 volume);
    u8 CalculateChecksum(u8 *aubData_p, int auwDataLength);
    void send_io_ctl(u8 can_id,st_led data);
    void handle_out_data(Data_out data);
private:
    int data_in_c;
	
};


#endif /* MOTOR_H_ */

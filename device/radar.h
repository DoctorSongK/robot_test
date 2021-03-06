#ifndef AGV_RADAR_H
#define AGV_RADAR_H

#include <pthread.h>
#include <map>

#include "../common/sensor_data.h"
#include "../common/msg_queue.h"
class RadarModule {
public:
    RadarModule();
    ~RadarModule();

    void set_target(MsgQueue * target, int channel);
    void send_collision_msg(int level);
    bool radar_disconnect = true;
    bool radar_error = false;
private:
    pthread_t monitor_radar_thread;
    static void* monitor_radar_state(void* param);
    int detect_cnt = 0;
    static void *thread_function(void *param);
    void close_radar();
	int laser_reconnect();
    pthread_t radar_thread;
    bool thread_running = true;

    pthread_mutex_t map_mutex;
    MsgQueue * p_msg_queue = nullptr;
    int channel;
    bool first_connect = true;
    //把其他的中间变量定义在这里
    //比如文件句柄，计数之类的
    //-----------------
    enum CMD {
        SINGLE_READ,
        CONTINUE_READ_START,
        CONTINUE_READ_STOP
    };
	int    sockfd;
    //char const *host_name ="192.168.192.200";//服务端IP
    //int port = 2112;

    char send_buf[1024];

    char const *cmd_single_read="sRN LMDscandata";
    char const *cmd_continue_read_start="sEN LMDscandata 1";
    char const *cmd_continue_read_stop="sEN LMDscandata 0";

    void make_cmd_frame(enum RadarModule::CMD cmd);
    void make_cmd_frame2(enum RadarModule::CMD cmd);
};

#endif
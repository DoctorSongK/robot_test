#ifndef AGV_TWO_DIM_H
#define AGV_TWO_DIM_H

#include <pthread.h>
#include "../common/two_dim_data.h"
#include <map>
#include "../common/msg_queue.h"
enum {
    TWO_NONE, TWO_CODE1, TWO_CODE2
} device_num;/*!< 设备编号 */
class Two_Dim_recModule {
public:
    Two_Dim_recModule();
    ~Two_Dim_recModule();

    bool device1_disconnect = true;
    bool device2_disconnect = true;
    bool td_code_1_error = false;
    bool td_code_2_error = false;
    void set_target(MsgQueue * target, int channel);
private:
    pthread_t monitor_2d_code_thread;
    static void* monitor_2d_code_state(void* param);
    int detect_2d_code1_cnt = 0;
    int detect_2d_code2_cnt = 0;

    int    sockfd_1;
    int    sockfd_2;
    bool thread1_running = true;
    bool thread2_running = true;
    static void *thread1_function(void *param);
    static void *thread2_function(void *param);
    void close_device();
    int device1_reconnect();
    int device2_reconnect();
    pthread_t device1_thread;
    pthread_t device2_thread;
    MsgQueue * p_msg_queue = nullptr;
    int channel;
    pthread_mutex_t map_mutex;
};

#endif
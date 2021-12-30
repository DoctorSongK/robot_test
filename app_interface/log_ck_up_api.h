#ifndef LOG_CK_UP_H
#define LOG_CK_UP_H

#include <termios.h>
#include <pthread.h>
#include <map>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include "myLog.h"
#include "../common/cJSON.h"
typedef enum  
{
    robot_status_info_req=1000,
    robot_status_run_req=1002,
    robot_status_mode_req,
    robot_status_loc_req, 
    robot_status_speed_req,
    robot_status_block_req,
    robot_status_battery_req,
    robot_status_brake_req,
    robot_status_laser_req,
    robot_status_path_req,
    robot_status_area_req,
    robot_status_emergency_req,
    robot_status_task_req=1020,
    robot_status_reloc_req,
    robot_status_loadmap_req,
    robot_status_slam_req=1025,
    robot_status_alarm_req=1050,
    robot_status_all1_req=1100,
    robot_status_all2_req,
    robot_status_all3_req,
    robot_status_init_req=1111,
    robot_status_map_req=1300,

    robot_control_stop_req=2000,
    robot_control_gyrocal_req,
    robot_control_reloc_req,
    robot_control_comfirmloc_req,
    robot_control_cancel_reloc_req,
    robot_control_motion_req=2010,
    robot_control_slam_req=2020,
    robot_control_endslam_req,
    robot_control_high_slam_req,
    robot_control_end_high_slam_req,

    robot_task_pause_req=3001,
    robot_task_resume_req,
    robot_task_cancel_req,
    robot_task_gohome_req=3030,
    robot_task_charge_req=3040,
    robot_task_gopoint_req=3050,
    robot_task_gotarget_req,
    robot_task_translate_req=3055,
    robot_task_turn_req,
    robot_task_antiepidemic_req,
    
    robot_config_carpara_req=4000,
    robot_config_carpara_upload_req,
    robot_config_car_drv_forward_req,
    robot_config_car_drv_backward_req,
    robot_config_car_drv_left_req,
    robot_config_car_drv_right_req,
    robot_config_car_drv_stop_req,
    robot_config_debug_fileup_req,
    robot_config_car_drv_zero_mov_req,
    robot_config_car_drv_line_mov_req,

    robot_config_uploadmap_req=4010,
    robot_config_downloadmap_req,
    robot_config_removemap_req,
    robot_config_map_filename_req,
    robot_config_load_ori_map_req,
    robot_config_map_path_disconnect_req,
    robot_config_ini_config_pull_req,
    robot_config_ini_config_push_req,
    robot_config_ini_config_move_req,
    robot_config_ini_config_move_up_req,
    robot_config_ini_config_avoid_req,
    robot_config_ini_config_avoid_up_req,
    robot_program_update_req,
    robot_obsmap_update_req,
    robot_update_reboot,
    robot_config_ini_get_control_req=4025,
    robot_config_ini_release_control_req,
    robot_config_car_drv_rot_stop=4050,
    robot_config_car_drv_speed_pull,
    robot_config_car_drv_speed_push,
    robot_config_cllision_switch,
    robot_config_lift_drv,

    robot_config_cap_points_pull=4060,
    robot_config_cap_points_push,
    robot_config_cap_points_catch,
    robot_points_cloud_pull,
    robot_config_path_pull,
    robot_config_path_push,
    robot_config_path_set,
    robot_config_path_run,
    robot_config_task_log_pull,
    robot_config_path_point_run,
    robot_config_rotate_point_run,
} ROBOT_CMD_REQ;///////////////机器人状态请求
typedef struct 
{
    char herder_syn;////同步头
    char version_num;/////版本号
    char serial_num1;/////编号1
    char serial_num2;/////编号2
    //int data_len;/////数据长度
    
    char data_len3;/////数据长度
    char data_len2;/////数据长度
    char data_len1;/////数据长度
    char data_len0;/////数据长度
    char cmdtype_HB;////命令类型高字节
    char cmdtype_LB;////命令类型低字节
    char reserver_data1;/////保留字节
    char reserver_data2;/////保留字节
    char reserver_data3;/////保留字节
    char reserver_data4;/////保留字节
    char reserver_data5;/////保留字节
    char reserver_data6;/////保留字节
}T_COMMU_HEADER;///////通信结构体头部
#define LOBYTE(w) ((char)(((int)(w))&0xff))
#define HIBYTE(w) ((char)((((int)(w))>>8)&0xff))
#define CUR_VERSION		"20210610"
#define UPDATE_PATH_NAME	"/usr/local/ale_bot/" //////bin 路径

typedef unsigned char     uint8;
//typedef unsigned long    m_uint32;
//定义编码字典


class LOG_CK_UPModule {
public:
    LOG_CK_UPModule();
    ~LOG_CK_UPModule();

    void set_target(MsgQueue * target, int channel);
    pthread_t main_thread;
    ssize_t my_write(int fd,void *buffer,size_t n);
    ssize_t      /* Read "n" bytes from a descriptor. */
    my_read(int fd, void *vptr, size_t n);
    int Port_task = 19205;
    int pox_system(const char *cmd_line); 
private:
    static void *thread_function(void *param);
    static void *task_robot_task(void* param);
    static void * pthread_func_task(void* p_client_sockfd);// pthread_func(int* p_client_sockfd)
    int SocketConnected(int sock);
    bool save_map_name(string name);
    int client_sockfd;
    bool thread_running = true;
    int buffer_analysize(int socket_hd,int recv_len,char *recv_buf,int port);
    //把其他的中间变量定义在这里
    MsgQueue * p_msg_queue = nullptr;
    int channel;
    pthread_mutex_t mutex;
    int byte2int(char *buf);
    
    std::string base64_encode(const char * bytes_to_encode, unsigned int in_len);
    std::string base64_decode(std::string const & encoded_string);
    inline bool is_base64(const char c)
    {        return (isalnum(c) || (c == '+') || (c == '/'));    }
    void int2byte(char *buf, int value);
 
};
void init_LOG_CK_UP_instance();
#endif


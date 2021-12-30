#ifndef WARN_ERR_H
#define WARN_ERR_H
#include <cmath>
#include <vector>

enum {
    NONE_OBS,  WARN_OBS_1, WARN_OBS_2,EMC_OBS,EMC_OBS_P
} laser_obs_state;/*!< 激光避障状态 */
/**
 * @brief SignalIn Externed
 */
typedef struct {
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
    //char _charging;////充电标识
    
} st_board_sts;
typedef struct {

    bool laser_error = false;
    bool laser_disconnect = true;
    bool motor_error = false;
    bool motor_disconnect = true;
    bool imu_disconnect = true;
    bool board_disconnect = true;
    bool _battary_warning = false;////电量报警
    bool pose_initilized = false; 
    bool collision_switch;
    bool device2_disconnect = true;
    bool device1_disconnect = true;
    char collision_level;
    bool lift_up_flag = false;
    bool lift_up_turn_switch = false;
    bool dm_code_error = false;
    char charging;/////
} st_device_sts;

typedef struct{
        int ahead_obstacle;//前方障碍物
        int rear_obstacle;//后方障碍物
        int left_obstacle;//左侧障碍物
        int right_obstacle;//右侧障碍物
}obavoid_state_3d;

class Warn_err_data {
public:
    Warn_err_data() : Warn_err_data(0, 0) {};
    Warn_err_data(char wc, char ec)
        : warn_code(wc), err_code(ec){
        
    };

    Warn_err_data & operator=(const Warn_err_data &p) {
        warn_code = p.warn_code;
        err_code = p.err_code;
        ctl_sts = p.ctl_sts;
        dev_sts = p.dev_sts;
        ob_3d_sts = p.ob_3d_sts;
        return *this;
    }
    char  warn_code;
    char  err_code;
    st_board_sts ctl_sts;
    st_device_sts dev_sts;
    obavoid_state_3d ob_3d_sts;
    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Warn_err_data));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Warn_err_data);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Warn_err_data)) {
            Warn_err_data *p = (Warn_err_data*)buf;
            warn_code = p->warn_code;
            err_code = p->err_code;
            ctl_sts = p->ctl_sts;
            dev_sts = p->dev_sts;
            ob_3d_sts = p->ob_3d_sts;

        }
    }
};
class Laser_obs_data {
public:
    Laser_obs_data() : Laser_obs_data(0) {};
    Laser_obs_data(char level)
        : laser_obs_level(level),laser_obs_level_l(level),laser_obs_level_p(level){
        
    };

    Laser_obs_data & operator=(const Laser_obs_data &p) {
        laser_obs_level = p.laser_obs_level;
        laser_obs_level_l = p.laser_obs_level_l;
        laser_obs_level_p = p.laser_obs_level_p;
        return *this;
    }
    char  laser_obs_level;
    char  laser_obs_level_l;////雷达避障
    char  laser_obs_level_p;////点激光避障
    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Laser_obs_data));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Laser_obs_data);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Laser_obs_data)) {
            Laser_obs_data *p = (Laser_obs_data*)buf;
            laser_obs_level = p->laser_obs_level;
            laser_obs_level_l = p->laser_obs_level_l;
            laser_obs_level_p = p->laser_obs_level_p;
        }
    }
};

class Laser_obs_3d_data {
public:
    Laser_obs_3d_data() : Laser_obs_3d_data(0,4,4,4,4,4) {};
    Laser_obs_3d_data(char p_level,char l_f_level,char l_b_level,char l_l_level,char l_r_level,char l_level)
        : laser_obs_level_p(p_level),laser_obs_level_f_l(l_f_level),laser_obs_level_b_l(l_b_level),laser_obs_level_l_l(l_l_level),laser_obs_level_r_l(l_r_level),laser_obs_level(l_level){
        
    };

    Laser_obs_3d_data & operator=(const Laser_obs_3d_data &p) {
        laser_obs_level_p = p.laser_obs_level_p;
        laser_obs_level_f_l = p.laser_obs_level_f_l;
        laser_obs_level_b_l = p.laser_obs_level_b_l;
        laser_obs_level_l_l = p.laser_obs_level_l_l;
        laser_obs_level_r_l = p.laser_obs_level_r_l;
        laser_obs_level_l = p.laser_obs_level_l;
        laser_obs_level = p.laser_obs_level;
        return *this;
    }
    char  laser_obs_level_p;////点激光避障
    char  laser_obs_level_f_l;////雷达前避障
    char  laser_obs_level_b_l;////雷达后避障
    char  laser_obs_level_l_l;////雷达左避障
    char  laser_obs_level_r_l;////雷达右避障
    char  laser_obs_level;////雷达避障
    char  laser_obs_level_l;
    
    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Laser_obs_3d_data));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Laser_obs_3d_data);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Laser_obs_3d_data)) {
            Laser_obs_3d_data *p = (Laser_obs_3d_data*)buf;
            laser_obs_level_p = p->laser_obs_level_p;
            laser_obs_level_f_l = p->laser_obs_level_f_l;
            laser_obs_level_b_l = p->laser_obs_level_b_l;
            laser_obs_level_l_l = p->laser_obs_level_l_l;
            laser_obs_level_r_l = p->laser_obs_level_r_l;
            laser_obs_level = p->laser_obs_level;
        }
    }
};

#endif


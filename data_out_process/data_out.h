#ifndef DATA_OUT_H
#define DATA_OUT_H
#include <cmath>
#include <vector>
enum {
    LIFT_NONE, LIFT_UP, LIFT_DOWN,LIFT_TURN,LIFT_TURN_ZERO,LIFT_ALL_ZERO,
} lift_state;/*!< 小车运行方式 */
/**
 * @brief  motion structure definition
 */
typedef struct {

    double lift_height; /*!< 0-顶升下降  1-顶升上升 */
    double lift_angle; /*!< 旋转角度 */
    float line_speed; /*!< 直线速度 */
    float angle_speed; /*!< 旋转速度 */
    char lift_state; /*!< 运动状态 0-无动作 1-上升  2-下降  3-旋转 */
} st_lift;

/**
 * @brief  motion structure definition
 */
typedef struct {

    char volume; /*!< 音量 */
    char index; /*!< 声音索引 */
    char run_state; /*!< 播放状态状态 0-无播放 1-播放  2-播放 */
} st_voice;
/**
 * @brief  motion structure definition
 */
typedef struct {

    float vel_l; /*!< 差分左轮速度 */
    float vel_r; /*!< 差分右轮速度 */
    float vel_f_steer; /*!< 前舵轮速度 */
    float vel_b_steer; /*!< 后舵轮速度 */
    float rot_f_steer; /*!< 前舵轮转角 */
    float rot_b_steer; /*!< 后舵轮转角 */
    char move_state; /*!< 运动状态 0-正常驱动 1-直行  2-原地 */
} st_motion;
/**
 * @brief SignalIn Externed
 */
typedef struct {

    char led_color;
    char io_extra;
    char led_state;
} st_led;


class Data_out {
public:
    Data_out() {};


    Data_out & operator=(const Data_out &p) {
        led_info = p.led_info;
        move_info = p.move_info;
        lift_info = p.lift_info;
        voice_info = p.voice_info;
        f_led_voice = p.f_led_voice;
        f_move = p.f_move;
        f_lift = p.f_lift;
        return *this;
    }
    st_led  led_info;
    st_motion  move_info;
    st_lift  lift_info;
    st_voice  voice_info;
    char f_led_voice;
    char f_move;
    char f_lift;

    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Data_out));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Data_out);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Data_out)) {
            Data_out *p = (Data_out*)buf;
            led_info = p->led_info;
            move_info = p->move_info;
            lift_info = p->lift_info;
            voice_info = p->voice_info;
            f_led_voice = p->f_led_voice;
            f_move = p->f_move;
            f_lift = p->f_lift;
        }
    }
};
#endif


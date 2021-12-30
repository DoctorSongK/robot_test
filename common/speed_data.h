#ifndef AGV_SPEEDD_H
#define AGV_SPEEDD_H
#include <cmath>
#include <vector>
/*************************************************这里舵轮AGV增加横移和蟹行运行方式*******************************************************/
enum {
    NORMAL_RUN, STRAIGHT, HOME_TURN, ACROSS_RUN, OBLIQUE_RUN
} run_state;/*!< 小车运行方式 */
enum {
    MANUAL_CMD, AUTO_CMD
} cmd_state;/*!< 指令类型 */
class Speed_data {
public:
    Speed_data() : Speed_data(0, 0.) {};
    Speed_data(double vl, double va)
        : vel_line(vl), vel_ang(va){
        while(vel_ang <- M_PI ) {
            vel_ang += (M_PI * 2);
        }
        while(vel_ang > M_PI ) {
            vel_ang -= (M_PI * 2);
        }
    };

    Speed_data & operator=(const Speed_data &p) {
        vel_line = p.vel_line;
        vel_ang = p.vel_ang;
        rot_rf = p.rot_rf;
        rot_rb = p.rot_rb;
        line_lf = p.line_lf;
        line_lb = p.line_lb;
        run_state = p.run_state;/////0-正常运行模式  1-切换直行 2-切换原地转
        oblique_angle = p.oblique_angle;
        _cmd_type = p._cmd_type;
        return *this;
    }
    double  vel_line;
    double  vel_ang;
    double  rot_rf;
    double  rot_rb;
    double  line_lf;
    double  line_lb; 
    /*************************************四舵轮*********************************************/
    double oblique_angle;
    double rot_rlf; //左前轮角速度
    double rot_rrf;//右前轮角速度
	double rot_rlb;//左后轮角速度
	double rot_rrb;//右后轮角速度
	double line_llf;//左前轮线速度
	double line_lrf;//右前轮线速度
	double line_llb;//左后轮线速度
	double line_lrb;//右后轮线速度
	/****************************************************************************************/
    char    run_state;
    char    _cmd_type;
    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Speed_data));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Speed_data);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Speed_data)) {
            Speed_data *p = (Speed_data*)buf;
            vel_line = p->vel_line;
            vel_ang = p->vel_ang;
            rot_rf = p->rot_rf;
            rot_rb = p->rot_rb;
            line_lf = p->line_lf;
            line_lb = p->line_lb;
            /*************************************四舵轮*********************************************/
            oblique_angle = p->oblique_angle;
            rot_rlf = p->rot_rlf;
            rot_rrf = p->rot_rrf;
            rot_rlb = p->rot_rlb;
            rot_rrb = p->rot_rrb;
            line_llf = p->line_llf;
            line_lrf = p->line_lrf;
            line_llb = p->line_llb;
            line_lrb = p->line_lrb;
    /****************************************************************************************/
            run_state = p->run_state;
            _cmd_type = p->_cmd_type;
        }
    }
};
#endif


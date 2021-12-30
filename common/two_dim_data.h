#ifndef AGV_TWO_DIM_DATA_H
#define AGV_TWO_DIM_DATA_H
#include <cmath>
#include <vector>

class Two_dim_data {
public:
    Two_dim_data() : Two_dim_data(0, 0,0,0,0) {};
    Two_dim_data(long time_us, float x, float y, float theta,int tagnum)
        :timestamp(time_us), agv_x(x),agv_y(y),agv_theta(theta), tag_num(tagnum){
        while(agv_theta <- M_PI ) {
            agv_theta += (M_PI * 2);
        }
        while(agv_theta > M_PI ) {
            agv_theta -= (M_PI * 2);
        }
    };

    Two_dim_data & operator=(const Two_dim_data &p) {
          timestamp = p.timestamp;
          agv_x = p.agv_x;
          agv_y = p.agv_y;
          agv_theta = p.agv_theta;
          tag_num = p.tag_num;
          device_send =  p.device_send;
          state = p.state;
        return *this;
    }
    long    timestamp;
    float agv_x;
    float agv_y;
    float agv_theta;
    int tag_num;
    int device_send;
    char state;
    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Two_dim_data));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Two_dim_data);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Two_dim_data)) {
            Two_dim_data *p = (Two_dim_data*)buf;
            timestamp = p->timestamp;
            agv_x = p->agv_x;
            agv_y = p->agv_y;
            agv_theta = p->agv_theta;
            tag_num = p->tag_num;
            device_send =  p->device_send;
            state = p->state;
        }
    }
};
#endif


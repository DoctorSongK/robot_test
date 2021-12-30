#ifndef CTL_DATA_H
#define CTL_DATA_H
#include <cmath>
#include <vector>
typedef unsigned char u8;

class Ctl_board_data {
public:
    Ctl_board_data() : Ctl_board_data(0, 0) {};
    Ctl_board_data(int wc, int ec)
        {
        
    };

    u8  data[8];

    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(u8)*8);
        char *p = (char*)this;
        for(int i = 0;i < sizeof(u8)*8;i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(u8)*8) {
            Ctl_board_data *p = (Ctl_board_data*)buf;
            data[0] = p->data[0];
            data[1] = p->data[1];
            data[2] = p->data[2];
            data[3] = p->data[3];
            data[4] = p->data[4];
            data[5] = p->data[5];
            data[6] = p->data[6];
            data[7] = p->data[7];
        }
    }
};

#endif


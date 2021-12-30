#include "broadcast.h"

Io_music_modual::Io_music_modual(MsgQueue *queue,  int data_iput)
{
    data_in_c = data_iput;
    this->queue = queue;
    queue->add_listener(data_in_c, this);
}


Io_music_modual::~Io_music_modual()
{
    queue->remove_listener(data_in_c, this);
}
void Io_music_modual::recieve_message(const int channel, char *buf, const int size)
{

    if (channel == data_in_c)
    {
        Data_out data;
        data.from_char_array(buf, size);
        handle_out_data(data);
    }
	
}
void Io_music_modual::handle_out_data(Data_out data)
{
    //if(data.f_led_voice == 0) return;

    //play_music(0,data.voice_info.index,data.voice_info.volume);
    //usleep(20000);
    send_io_ctl(0x09,data.led_info);
}
void Io_music_modual::play_music(u8 can_id,u8 index,u8 volume)
{
    //if(can_handle<=0) return;
    int ret;
    u8 data[8] = {0x01, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
    data[2] = index;
    if(volume<10) volume = 10;
    if(volume>28) volume = 28;
    data[3] = volume;
    data[6] = CalculateChecksum(data,6);
    //ret = can_send(1, (0x200 + can_id), data);

}
void  Io_music_modual::send_io_ctl(u8 can_id,st_led out)
{
    //if(can_handle<=0) return;
    int ret;
    u8 data[8] = {0x01, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
    data[0] = out.led_color;
    data[1] = out.io_extra;
    data[2] = out.led_state;

    //ret = can_send(1, (can_id), data);
    //printf("io data_send \n");
    //ret = can_send(1, (0x200 + can_id), data);

}
u8 Io_music_modual::CalculateChecksum(u8 *aubData_p, int auwDataLength)
{
    u8 aubChecksum = 0;
    int auwCnt = 0;
 
    while(auwCnt < auwDataLength)
    {
        aubChecksum ^= aubData_p[auwCnt];
        auwCnt++;
    }
 
    return aubChecksum;
}

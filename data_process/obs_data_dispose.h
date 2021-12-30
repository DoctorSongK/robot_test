#ifndef OBS_DISPOSE_H
#define OBS_DISPOSE_H
#include "../common/warn_err_data.h"
#include "../common/msg_queue.h"

class Obs_dispose : public MsgQueueListener {
public:
    Obs_dispose(MsgQueue *queue,  int war_err_chan, int radar_chan,int radar_3d_sts_chan);
    ~Obs_dispose();

    int handle_radar_data(PointCloudData & data);
    int handle_sts_data(Warn_err_data & data);
    int handle_radar_3d_data(Laser_obs_3d_data& ob_data);
    virtual void recieve_message(const int channel, char *buf, const int size);
	
    MsgQueue *  queue;
    void begin_high_resolution_map_create();
    void end_high_resolution_map_create();	
         st_device_sts device_sts;
	st_board_sts board_sts;
    obavoid_state_3d radar_3d_sts;

private:
    FILE *map_raw_data_file;////未处理地图数据文件
    int record_order;
    bool first_record;
    Position last_record_pos;
    bool begin_mapping = false;
    void _recor_data2map();
    Position radar_pos;
    Position radar_raw_pos;

    PointCloudData last_radar_data;
    int war_err_c;
    int radar_c;
    int radar_3d_sts_c;
    char laser_collision_level;
    char point_laser_collision_level;
    char collision_level;
    double l1x;
    double l1y;
    double l2x;
    double l2y;
    double l3x;
    double l3y;
    int thsh ;
    bool collision_switch;
   
    int colli_cnt1;
    int colli_cnt2;
    int colli_cnt3;
    int radar_cnt = 0;
    int use_radar_type;
};

#endif

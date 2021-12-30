#ifndef STATUS_DISPOSE_H
#define STATUS_DISPOSE_H
#include "../common/warn_err_data.h"
#include "../data_out_process/broadcast.h"
#include "../common/msg_queue.h"

class Status_dispose : public MsgQueueListener {
public:
    Status_dispose(MsgQueue *queue,  int robot_sts_chan, int radar_obs_chan);
    ~Status_dispose();

    int handle_sts_data(Warn_err_data & data);
    virtual void recieve_message(const int channel, char *buf, const int size);
	
    MsgQueue *      queue;
	
private:
    int robot_sts_c;
    int radar_obs_c;
    char collision_level;
    st_device_sts device_sts;
	st_board_sts board_sts;
    obavoid_state_3d ob_3d_sts;
    char warning_code = 0;
    char error_code = 0;
    bool collision_switch;
    int warning_bat;
    int use_radar_type;
};

#endif

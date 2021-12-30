#ifndef LOCATION_H
#define LOCATION_H
#include <assert.h>
#include <vector>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <termios.h>
#include "../location/amcl_gl.h"
#include "scan_matching.h"
#include "../common/simple_grid_map.h"
#include "../device/two_dim_rec.h"
#include "../location/pose_correct.h"
#include "../device/radar.h"
#include "../common/pose.h"
#include "../common/time.h"
#include "../common/capture_point.h"
#include "../common/msg_queue.h"
#include "../common/warn_err_data.h"

#define SYSTEM_STATE_OFFLINE    0
#define SYSTEM_STATE_FREE       1
#define SYSTEM_STATE_MAPPING    2
#define SYSTEM_STATE_LOCATING   3
#define SYSTEM_STATE_NAVIGATING 4
#define SYSTEM_STATE_PAUSE      5
#define SYSTEM_STATE_OPTIMISM   6
#define SYSTEM_STATE_INIT_POSE  7
#define SYSTEM_STATE_OPERATION  8
#define SYSTEM_STATE_MAPPING_LANDMARK  2

class MapSharedMemory {
public:
    MapSharedMemory(MapInfo &info) : map(new SimpleGridMap(info)) {
        pthread_mutex_init(&mutex, nullptr);
    }
    MapSharedMemory(SimpleGridMap &m) : map(new SimpleGridMap(m)) {
        pthread_mutex_init(&mutex, nullptr);
    }
    MapSharedMemory(char * data_buf, int buf_size) : map(new SimpleGridMap(data_buf, buf_size)) {
        pthread_mutex_init(&mutex, nullptr);
    }
    ~MapSharedMemory() {
        pthread_mutex_destroy(&mutex);
        delete map;
    }
    void increase(int i) {
        pthread_mutex_lock(&mutex);
        count += i;
        pthread_mutex_unlock(&mutex);
    }
    bool free(int i) {
        bool ret = false;
        pthread_mutex_lock(&mutex);
        count -= i;
        ret = (count < 1);
        pthread_mutex_unlock(&mutex);
        return ret;
    }
    pthread_mutex_t mutex;
    SimpleGridMap *map;
    int count = 0;
};

class MapSharedPointer {
public:
    MapSharedPointer() {};
    MapSharedPointer(MapSharedMemory * p_mem):mem(p_mem) {
        if(mem != nullptr) {
            mem->increase(1);
        }
    };
    MapSharedPointer(MapSharedPointer &ptr) : MapSharedPointer(ptr.mem) {};
    MapSharedPointer& operator=(const MapSharedPointer& ptr) {
        if(&ptr != this) {
            mem = ptr.mem;
            if(mem != nullptr) {
                mem->increase(1);
            }
        }
        return *this;
    };
    MapSharedPointer& operator=(MapSharedMemory * p_mem) {
        if(mem != nullptr) {
            if(mem->free(1)) {
                delete mem;
            }
        }
        mem = p_mem;
        if(mem != nullptr) {
            mem->increase(1);
        }
        return *this;
    };

    ~MapSharedPointer() {
        if(mem != nullptr) {
            if(mem->free(1)) {
                delete mem;
            }
        }
    };

    bool is_nullptr() {
        return mem == nullptr;
    }
    SimpleGridMap *get() {
        return mem->map;
    }
    SimpleGridMap &operator*() {
        return *(mem->map);
    };
    SimpleGridMap *operator->() {
        return mem->map;
    }

private:
    MapSharedMemory *mem = nullptr;
};

typedef struct {
    float pose[3];
    std::vector<double> points;
} LandmarkRadarData;
typedef unsigned char byte;

class Core_location : public MsgQueueListener {
public:
	Core_location(MsgQueue *queue, int radar_chan,int odom_chan);
    virtual void recieve_message(const int channel, char *buf, const int size);
	~Core_location();
    void registe_listener();
public:

    void update_last_position(Position &pose);
    void update_last_position_raw(Position &pose);
    Position expolate_current_position_tim(long timestamp);
    Position expolate_current_position();
    void add_odom_data(Position &pose_change);
    void add_odom_data_raw(Position &pose_change);
    void update_odom_list_timestamp(long timestamp_us);
    void update_odom_list_timestamp_raw(long timestamp_us);

    void flush_odom_data();
    void flush_odom_data_raw();
    
    Position expolate_current_position_tim_raw(long timestamp);
    Position expolate_current_position_raw();

    void add_odom_data_robot(Position &pose_change);
    void flush_odom_data_robot();
    Position expolate_current_position_robot();
    void update_last_position_robot(Position &pose);
    void update_odom_list_timestamp_robot(long timestamp_us);

    Amcl_gl * amcl_global_locate;
    void do_amcl_global_locating();

    int record_pose_cnt = 0;
    bool pose_initilized = false;

    bool scan_init;///////////////lwglwg20201210
    int insert_index;////////////lwg20201210
    raw_pos_for_correct raw_pose;////////////lwg20201210

    int robot_stop_cnt = 0;///
    int on_scan_matching_cnt_;////
	int scan_matching_cnt;
    bool scan_matching_pause = false;
    bool doing_scan_matching = false;

    bool land_mark_mapping=false;

    Position do_scan_matching(PointCloudData &data, Position &pose, char * config_item);
    //void do_scan_matching(PointCloudData &data);
    void do_scan_matching(PointCloudData &data,Position &pos_ret ,bool &is_correct);///////////////////20201210LWG

    std::list<Two_dim_data> total_two_dims;////the total two dimensiton code in the map
    int set_two_dim_data(std::list<Two_dim_data> two_in);////set the two dim list
    int use_two_dim_data_correct_robpos(Two_dim_data &two_d);////us the two dim data to correct the robotpos
    bool two_dims_add(Two_dim_data &two_d);


	Position pose_from_save_file;
    bool loadPose(double xp,double yp,double theta);
    bool loadPoseFromserver();
    string map_name_saved;
	pthread_t initial_locate_thread;///
    static void * initial_locate_thread_function(void * param);  ////
	bool init_locate();////
    void start_global_locating();
    pthread_t global_amcl_locate_thread;
    static void * global_amcl_locate_thread_function(void *param);

    void _begin_map_create();
    void _end_map_create();
    void load_grid_map(char *buf, int size);
    void load_grid_map(SimpleGridMap &map);

    MapSharedPointer p_map;
    int state = SYSTEM_STATE_OFFLINE;
    MapSharedPointer get_current_map() {return p_map;};

    bool start_mapping_landmark();
    bool stop_mapping_landmark();
    int get_cur_state();
    bool Set_dm_code_correct(bool input);

    double total_odom;
    double today_odom = 0;
    void load_map_name();
    void set_map_name(string map_name);
    st_device_sts agv_sts;
private:
    bool use_dm_code_correct = false;
    MsgQueue *queue_a;
    //bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...
    int radar_c;
    int odom_c;
    pthread_mutex_t listen_pose_mutex;
    Position radar_position;
    int last_dm_index = -1;
    int pose_level = 1;
    Position pose2d;
    pthread_mutex_t mutex_pose;
    std::list<Position> odom_list;
    pthread_mutex_t mutex_odom_list;
    
    Position pose2d_raw;
    pthread_mutex_t mutex_pose_raw;
    std::list<Position> odom_list_raw;
    pthread_mutex_t mutex_odom_list_raw;

    Position pose2d_robot;
    pthread_mutex_t mutex_pose_robot;
    std::list<Position> odom_list_robot;
    pthread_mutex_t mutex_odom_list_robot;

    pthread_mutex_t mutex_state;
    pthread_mutex_t mutex_map;
   
    pthread_mutex_t mutex_dm_correct;
    
public:
    void set_saved_pos(Position pos_save);
    char * My_strsub_i2e(char * src,int start_i,int len)
    {
    assert(src != NULL);
    int total_length = start_i+len;//
    int length = total_length - start_i;
    int real_length = ((length) >= 0 ? length : total_length)+1;
    char *tmp;
    //char *ret;
    if (NULL == (tmp=(char*) malloc(real_length * sizeof(char)))) {
        return NULL;
    }
    strncpy(tmp, src+start_i, real_length - 1);
    tmp[real_length - 1] = '\0';
    return tmp;	
    }
};
/////////////////////////////////////////////////////////////////////////////////






#endif

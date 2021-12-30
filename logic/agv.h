#ifndef AGV_INSTANCE_H
#define AGV_INSTANCE_H

#include <stdio.h>
#include <pthread.h>
#include <list>

#include "../location/location.h"
#include "../data_process/cmd_data_dispose.h"
#include "../location/carto.h"
#include "../device/odometer.h"
#include "../device/imu.h"
#include "../common/warn_err_data.h"
#include "storage.h"
#include "../location/pallet_recgnize.h"
#include "curve_run_fn.h"
#include "logic_process.h"
#include "../device/motor.h"
#include "../device/two_dim_rec.h"
#include "../common/cJSON.h"
#include "../device/control_board.h"
#include "antiepidemic_dispose.h"
#include "../data_process/obs_data_dispose.h"
#include "../data_process/status_data_dispose.h"
#include "../data_process/point_cloud_3D_data.h"
#include "../data_process/ridar_to_obavoid.h"
#include "../device/radar3D.h"
#define lift_up_done 0xc103
#define lift_turn_done 0xc104
#define traj_done 0xc101
class AGVInstance : public MsgQueueListener {

public:
    AGVInstance() {initialize();};
    AGVInstance(const AGVInstance &) = delete;
    AGVInstance& operator=(const AGVInstance&) = delete;
    ~AGVInstance() {shutdown();};

    int get_system_state();

    bool use_two_dim_correct_pallet(Position delt_pos);
    Position expolate_current_position();
    Position expolate_current_position_tim(long timestamp);
    Position expolate_current_position_tim_raw(long timestamp);
    Position expolate_current_position_raw();


	double obtain_imu_angle();
    MapSharedPointer get_current_map() {return locate_modual->p_map;};
    void load_grid_map(SimpleGridMap &map);
    void load_grid_map(char *buf, int size);

    MsgQueue * get_message_queue() {return message_queue;};

    virtual void recieve_message(const int channel, char *buf, const int size);

    bool start_mapping();
    bool stop_mapping();
    bool start_mapping_landmark();
    bool stop_mapping_landmark();

    bool start_navigating(CapRoute *r);
    bool start_navigating(RouteChain c);
    bool pause_navigating();
    bool stop_navigating();
	int exec_chain;
	int exec_route;
	int exec_cap;
	//mcu状态
	int mcu_input;
	int mcu_output;
	int mcu_battery_level;
	int mcu_speed;
	int mcu_omega;

    void start_global_locating();
    // void start_global_locating(double x, double y);

    PointCloudData get_last_radar_data();

    std::list<CapPoint> caps;
    std::list<CapRoute> routes;
    std::list<RouteChain> chains;
    std::list<Position> masks;
	double mask_radius;
	bool mask_switch;
    bool collision_switch=true;
    bool nav_thread_running = false;
	
    ROUTE r[ROUTE_SLOT_NUM];
	int map_id;
	char map_name[200];
	storage *st;
	int collision_level=0;
	int colli_cnt1=0;
	int colli_cnt2=0;
	int colli_cnt3=0;
	
	int default_locate_map_id;

    bool dm_code_record = false;
    bool load_two_dims_file();////load the saved two dim file
    bool start_two_dims_create();
    bool end_two_dims_create();
    bool two_dims_add(Two_dim_data &two_d);////insert the current two dim code that recognized
    Position pallet_bias_pos;////the bias pose of the pallet,recognized from the two dimention code
    bool b_two_dim_correct = false;
    std::list<Two_dim_data> total_two_dims;////the total two dimensiton code in the map
    bool loadPoseFromserver();/////lwg20201218灏嗘満鍣ㄤ汉浣嶅Э浠庢枃浠朵腑鍔犺浇
    bool loadPose(double xp,double yp,double theta);


	int pallet_recognize(float len,float width,float angle_sech,int clc_cnt,Position & result_pose);////锟斤拷锟斤拷位锟斤拷识锟斤拷
	int legs_recognize(float len,float width,float angle_sech,int clc_cnt,Position & leg1_pose,Position & leg2_pose);////锟斤拷锟斤拷锟剿诧拷识锟斤拷
	int start_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverset,char doact,vector <char> is_occ,vector <char> inv_cps,vector <char> atcs,vector <float> angles);////ret -1 failed; 1 success;doact(0-no action,1-up,2-down)
	bool cancel_path();
	bool stop_path();
	bool continue_path();
    bool reset_motor();////seset the motor
    bool laser_error = false;
	bool laser_disconnect = true;////
    bool imu_disconnect = true;
    bool motor_disconnect = true;
    bool motor_error = false;
    bool board_disconnect = true;

    st_board_sts board_sts;
    st_device_sts device_sts;
    obavoid_state_3d ob_3d_sts;
    
    bool lift_up_turn_switch = true;/////椤跺崌鍦ㄤ笂鏃嬭浆鏍囪瘑
    bool _battary_warning;/////鐢甸噺鎶ヨ
    char warning_code = 0;
    char error_code = 0;
    bool lift_complete;///椤跺崌鍔ㄤ綔瀹屾垚
    void update_agv_status();
    int cur_tagnum_locate;
    int cur_tagnum_pallet;
    void ChangeAgv2Nav();/////褰撴帴鏀跺埌杞崲鍒皀av妯″紡
private:
    void initialize();
    void shutdown();
    void registe_listener();
    int record_pose_cnt = 0;
    int recor_file_index = 1;
    bool savePoseToServer();
    float second_locate_max_val = 0.5;
    int state = SYSTEM_STATE_OFFLINE;
    pthread_mutex_t mutex_state;
    pthread_mutex_t mutex_dm_correct;
    MsgQueue * message_queue = nullptr;

    Position pose2d;
    int pose_level = 1;

    PointCloudData last_radar_data;
    pthread_mutex_t mutex_radar_data;

    std::list<Position> odom_list;
    pthread_mutex_t mutex_odom_list;
    void add_odom_data(Position &pose_change);
    void update_odom_list_timestamp(long timestamp_us);

    void do_global_locating();

    Pallet_recgnize * pallet_pose_recgnize;

    static void * global_locate_thread_function(void * param);
    pthread_t global_locate_thread;
    static void * painting_map_thread_function(void * param);
    pthread_t painting_map_thread;
    bool painting_map_thread_running = false;

    static void * io_control_thread_function(void * param);
    pthread_t io_control_thread;

    bool land_mark_mapping=false;
    std::list<LandmarkRadarData> landmark_radar_datas;
    int landmark_radar_data_insert_count = 0;
    void handle_landmark_data_on_painting(PointCloudData &data);

    // static void * navigating_thread_function(void * param);
    // pthread_t navigating_thread;
    // bool navigating_thread_running = false;
    // bool navigating_thread_pause = false;

	Position radar_position;

	IMUModule *imu = nullptr;

    //DevicesModule * devs = nullptr;
    RadarModule * radar = nullptr;
    Odometer *odom = nullptr;
    RadarModule3D *radar3d = nullptr;
    Point_cloud_3D_data *point3d = nullptr;
    AGV_Obavoid *ob_3d = nullptr;

    CartoModule * carto = nullptr;
    NavModule * nav = nullptr;
    
    //SlamKarto * my_karto = nullptr;
    Motor *motor =nullptr;
    Control_board *control_board_ = nullptr;
    Obs_dispose   *obs_data_dispose = nullptr;
    Status_dispose   *status_data_dispose = nullptr;
    Io_music_modual  *io_broad = nullptr;
public:
    Logic_process *logic_out_process = nullptr;
    Two_Dim_recModule *two_dim_ = nullptr;
    Core_location *locate_modual = nullptr;
    curve_run *my_runcurve = nullptr;
    Antiepidemic_modual robot_Antiepidemic;
};

void init_global_agv_instance();
AGVInstance * get_global_agv_instance();
storage *get_global_storage();
#endif

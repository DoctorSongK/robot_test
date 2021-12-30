#include <cmath>
#include <memory>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include "location.h"
#include "../common/configuration.h"
#include "../common/cJSON.h"
//#include "../logic/agv.h"
FILE *fp_scanmatching_pos;
Core_location::Core_location(MsgQueue *queue, int radar_chan,int odom_chan)
{
    
    this->queue_a = queue;
    radar_c = radar_chan;
    odom_c = odom_chan;
    on_scan_matching_cnt_  =-10;
    //record_pose_cnt = 0;
    pose_initilized = false;
    robot_stop_cnt = 128;
	fp_scanmatching_pos = fopen("dr_pos.txt","w");

    pthread_mutex_init(&listen_pose_mutex, nullptr);
    pthread_mutex_init(&mutex_pose, nullptr);
    pthread_mutex_init(&mutex_odom_list, nullptr);
    pthread_mutex_init(&mutex_pose_raw, nullptr);
    pthread_mutex_init(&mutex_odom_list_raw, nullptr);
    pthread_mutex_init(&mutex_pose_robot, nullptr);
    pthread_mutex_init(&mutex_odom_list_robot, nullptr);
    pthread_mutex_init(&mutex_state, nullptr);
    pthread_mutex_init(&mutex_map, nullptr);
    pthread_mutex_init(&mutex_dm_correct, nullptr);
	double radar_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
    double radar_y = get_configs()->get_float("radar", "radar_position_y", nullptr);
    double radar_theta = get_configs()->get_float("radar", "radar_position_theta", nullptr);

	radar_position.x = radar_x;
	radar_position.y = radar_y;
	radar_position.theta = radar_theta;
    //loadPoseFromserver();
    raw_pose.odom_raw_pos.reserve(10);
    raw_pose.scan_match_pos.reserve(10);
    raw_pose.amcl_pos.reserve(10);
    raw_pose.land_mark_pos.reserve(10);
    raw_pose.two_dims_pos.reserve(10);  
	registe_listener();
	amcl_global_locate = new Amcl_gl(queue,CHANNEL_RADAR,CHANNEL_ODOM);
    pthread_create(&initial_locate_thread, NULL, initial_locate_thread_function, this);
  
}
void Core_location::registe_listener()
{
    queue_a->add_listener(CHANNEL_POSE, this);
    queue_a->add_listener(CHANNEL_RADAR, this);
    queue_a->add_listener(CHANNEL_ODOM, this);
    queue_a->add_listener(CHANNEL_TWO_DIMENTION,this);
    queue_a->add_listener(CHANNEL_ROBOT_STS_OUT,this);
    // message_queue->print_debug_string();
}
#define DEL(p)        \
    if (p != nullptr) \
    {                 \
        delete p;     \
        p = nullptr;  \
    }
Core_location::~Core_location()
{
	DEL(amcl_global_locate);
    pthread_mutex_destroy(&listen_pose_mutex);
    pthread_mutex_destroy(&mutex_pose);
    pthread_mutex_destroy(&mutex_odom_list);
    pthread_mutex_destroy(&mutex_pose_raw);
    pthread_mutex_destroy(&mutex_odom_list_raw);
    pthread_mutex_destroy(&mutex_pose_robot);
    pthread_mutex_destroy(&mutex_odom_list_robot);
    pthread_mutex_destroy(&mutex_state);
    pthread_mutex_destroy(&mutex_map);
    pthread_mutex_destroy(&mutex_dm_correct);
    queue_a->remove_listener(CHANNEL_POSE, this);
    queue_a->remove_listener(CHANNEL_RADAR, this);
    queue_a->remove_listener(CHANNEL_ODOM, this);
    queue_a->remove_listener(CHANNEL_TWO_DIMENTION,this);
    queue_a->remove_listener(CHANNEL_ROBOT_STS_OUT,this);
}

void Core_location::add_odom_data_raw(Position &pose_change)
{
    long timestamp = pose_change.timestamp;
    pthread_mutex_lock(&mutex_pose_raw);
    if (timestamp < pose2d_raw.timestamp)
    {
    printf("000000timestamp is too small  exit\n");
    pthread_mutex_unlock(&mutex_pose_raw);
    //return;
    }
    pthread_mutex_lock(&mutex_odom_list_raw);
    if (odom_list_raw.empty())
    {
        odom_list_raw.push_back(pose_change);
    }
    else
    {
        for (auto iter = odom_list_raw.begin();; iter++)
        {
            if (iter == odom_list_raw.end() || iter->timestamp >= timestamp)
            {
                odom_list_raw.insert(iter, pose_change);
                break;
            }
        }
    }
    pthread_mutex_unlock(&mutex_odom_list_raw);
    pthread_mutex_unlock(&mutex_pose_raw);
}
void Core_location::add_odom_data_robot(Position &pose_change)
{
    long timestamp = pose_change.timestamp;
    pthread_mutex_lock(&mutex_pose_robot);
    if (timestamp < pose2d_robot.timestamp)
    {
    printf("3333timestamp is too small  exit\n");
    pthread_mutex_unlock(&mutex_pose_robot);
    //return;
    }
    pthread_mutex_lock(&mutex_odom_list_robot);
    if (odom_list_robot.empty())
    {
        odom_list_robot.push_back(pose_change);
    }
    else
    {
        for (auto iter = odom_list_robot.begin();; iter++)
        {
            if (iter == odom_list_robot.end() || iter->timestamp >= timestamp)
            {
                odom_list_robot.insert(iter, pose_change);
                break;
            }
        }
    }
    pthread_mutex_unlock(&mutex_odom_list_robot);
    pthread_mutex_unlock(&mutex_pose_robot);
}

void Core_location::add_odom_data(Position &pose_change)
{
    long timestamp = pose_change.timestamp;
    pthread_mutex_lock(&mutex_pose);
    if (timestamp < pose2d.timestamp)
    {
        printf("111111timestamp is too small  exit\n");
        pthread_mutex_unlock(&mutex_pose);
        //return;
    }
    pthread_mutex_lock(&mutex_odom_list);
    if (odom_list.empty())
    {
        odom_list.push_back(pose_change);
    }
    else
    {
        for (auto iter = odom_list.begin();; iter++)
        {
            if (iter == odom_list.end() || iter->timestamp >= timestamp)
            {
                odom_list.insert(iter, pose_change);
                break;
            }
        }
    }
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
}
void Core_location::update_odom_list_timestamp_raw(long timestamp_us)
{
    pthread_mutex_lock(&mutex_odom_list_raw);
    while (!odom_list_raw.empty())
    {
        if (odom_list_raw.front().timestamp > timestamp_us)
        {
            break;
        }
        odom_list_raw.pop_front();
    }
    pthread_mutex_unlock(&mutex_odom_list_raw);
}
void Core_location::update_odom_list_timestamp_robot(long timestamp_us)
{
    pthread_mutex_lock(&mutex_odom_list_robot);
    while (!odom_list_robot.empty())
    {
        if (odom_list_robot.front().timestamp > timestamp_us)
        {
            break;
        }
        odom_list_robot.pop_front();
    }
    pthread_mutex_unlock(&mutex_odom_list_robot);
}
void Core_location::update_odom_list_timestamp(long timestamp_us)
{
    pthread_mutex_lock(&mutex_odom_list);
    while (!odom_list.empty())
    {
        if (odom_list.front().timestamp > timestamp_us)
        {
            break;
        }
        odom_list.pop_front();
    }
    pthread_mutex_unlock(&mutex_odom_list);
}
void Core_location::flush_odom_data()
{
    pthread_mutex_lock(&mutex_pose);
    Position pose = pose2d;
    pthread_mutex_lock(&mutex_odom_list);
    if (pose_level <= 1)
    {
        while (!odom_list.empty())
        {
            pose = pose * odom_list.front();
            odom_list.pop_front();
        }
        pose2d = pose;
    }
    else
    {
        odom_list.clear();
    }
	//printf("fffffffffffffffffffffffffffffffcur pose is theta=%f\nddddddddddddddddddddddddddd",pose2d_raw.theta);
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
}
void Core_location::flush_odom_data_raw()
{
    pthread_mutex_lock(&mutex_pose_raw);
    Position pose = pose2d_raw;
    pthread_mutex_lock(&mutex_odom_list_raw);
    if (pose_level <= 1)
    {
        while (!odom_list_raw.empty())
        {
            pose = pose * odom_list_raw.front();
            odom_list_raw.pop_front();
        }
        pose2d_raw = pose;
    }
    else
    {
        odom_list_raw.clear();
    }
	//printf("qqqqqqqqqqqqqqqqqqqqqqqqqqqqcur pose is theta=%f\nddddddddddddddddddddddddddd",pose2d_raw.theta);
    pthread_mutex_unlock(&mutex_odom_list_raw);
    pthread_mutex_unlock(&mutex_pose_raw);
}
void Core_location::flush_odom_data_robot()
{
    pthread_mutex_lock(&mutex_pose_robot);
    Position pose = pose2d_robot;
    pthread_mutex_lock(&mutex_odom_list_robot);
    if (pose_level <= 1)
    {
        while (!odom_list_robot.empty())
        {
            pose = pose * odom_list_robot.front();
            odom_list_robot.pop_front();
        }
        pose2d_robot = pose;
    }
    else
    {
        odom_list_robot.clear();
    }
	//printf("qqqqqqqqqqqqqqqqqqqqqqqqqqqqcur pose is theta=%f\nddddddddddddddddddddddddddd",pose2d_raw.theta);
    pthread_mutex_unlock(&mutex_odom_list_robot);
    pthread_mutex_unlock(&mutex_pose_robot);
}
Position Core_location::expolate_current_position_tim(long timestamp)
{
    Position pos_temp;
    pthread_mutex_lock(&mutex_pose);
    Position ret = pose2d;
    
    pthread_mutex_lock(&mutex_odom_list);
    for (Position &p : odom_list)
    {
        if (p.timestamp > ret.timestamp && (p.timestamp <= timestamp))
        {
        ret = ret * p;
        }

    }
    pose2d = ret;
    while (!odom_list.empty())
    {
        if (odom_list.front().timestamp > timestamp)
        {
            break;
        }
        odom_list.pop_front();
    }
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
    return ret;
}

void Core_location::update_last_position(Position &pose)
{
    pthread_mutex_lock(&mutex_pose);
        if (pose.timestamp > pose2d.timestamp)
        {
            pose2d = pose;
            update_odom_list_timestamp(pose.timestamp);
        }
    pthread_mutex_unlock(&mutex_pose);
}
void Core_location::update_last_position_raw(Position &pose)
{
    pthread_mutex_lock(&mutex_pose_raw);
        if (pose.timestamp > pose2d_raw.timestamp)
        {
            pose2d_raw = pose;
            update_odom_list_timestamp_raw(pose.timestamp);
        }
    pthread_mutex_unlock(&mutex_pose_raw);
}
void Core_location::update_last_position_robot(Position &pose)
{
    pthread_mutex_lock(&mutex_pose_robot);
        if (pose.timestamp > pose2d.timestamp)
        {
            pose2d_robot = pose;
            update_odom_list_timestamp_robot(pose.timestamp);
        }
    pthread_mutex_unlock(&mutex_pose_robot);
}
Position Core_location::expolate_current_position_tim_raw(long timestamp)
{
    pthread_mutex_lock(&mutex_pose_raw);
    Position ret = pose2d_raw;
    
    pthread_mutex_lock(&mutex_odom_list_raw);
    for (Position &p : odom_list_raw)
    {
        if (p.timestamp > ret.timestamp && (p.timestamp <= timestamp))
        {
        ret = ret * p;
        }

    }
    pose2d_raw = ret;
    while (!odom_list_raw.empty())
    {
        if (odom_list_raw.front().timestamp > timestamp)
        {
            break;
        }
        odom_list_raw.pop_front();
    }
    pthread_mutex_unlock(&mutex_odom_list_raw);
    pthread_mutex_unlock(&mutex_pose_raw);
    return ret;
}
Position Core_location::expolate_current_position()
{
    Position pos_temp;
    pthread_mutex_lock(&mutex_pose);
    Position ret = pose2d;
    pthread_mutex_lock(&mutex_odom_list);
    for (Position &p : odom_list)
    {
        if (p.timestamp > ret.timestamp)
        {
            ret = ret * p;
        }
    }
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
    return ret;
}
Position Core_location::expolate_current_position_raw()
{
    pthread_mutex_lock(&mutex_pose_raw);
    Position ret = pose2d_raw;
    pthread_mutex_lock(&mutex_odom_list_raw);
    for (Position &p : odom_list_raw)
    {
        if (p.timestamp > ret.timestamp)
        {
            ret = ret * p;
        }
    }
    pthread_mutex_unlock(&mutex_odom_list_raw);
    pthread_mutex_unlock(&mutex_pose_raw);
    return ret;
}
Position Core_location::expolate_current_position_robot()
{
    pthread_mutex_lock(&mutex_pose_robot);
    Position ret = pose2d_robot;
    pthread_mutex_lock(&mutex_odom_list_robot);
    for (Position &p : odom_list_robot)
    {
        if (p.timestamp > ret.timestamp)
        {
            ret = ret * p;
        }
    }
    pthread_mutex_unlock(&mutex_odom_list_robot);
    pthread_mutex_unlock(&mutex_pose_robot);
    return ret;
}
int Core_location::get_cur_state()
{
    //pthread_mutex_lock(&mutex_state);
    int ret = state;
    //pthread_mutex_unlock(&mutex_state);
    return ret;
}
void Core_location::recieve_message(const int channel, char *buf, const int size)
{   
    Position odom_temp;

	if (channel == CHANNEL_POSE)
    {
        Position pose;
        pose.from_char_array(buf, size);
        if ( get_cur_state() == SYSTEM_STATE_MAPPING||( get_cur_state() == SYSTEM_STATE_INIT_POSE))//////如果是建图模式，则使用建图过程中的scan-match位姿修正里程计。
        if (!get_current_map().is_nullptr())
        update_last_position(pose);
    }
    if (channel == CHANNEL_TWO_DIMENTION)////excute the two dimention locate
    {
        Two_dim_data dim_data;
        dim_data.from_char_array(buf,size);
        if(dim_data.device_send == TWO_CODE1)////if received the locating two dimention code
        {
        //if(last_dm_index)////if mapping the two dimention
        /////use the two dim data to correct the robot position
        pthread_mutex_lock(&mutex_dm_correct);
        use_two_dim_data_correct_robpos(dim_data);
        pthread_mutex_unlock(&mutex_dm_correct);
        }
    }

    if (channel == CHANNEL_RADAR)
    {//得到scanmatch

        Position pose_scan;
        Position pose_amcl_;
        Position pose_scan_correct;
        Position pose_predict;
        bool pos_correct=false;
        Position  cur_pos = expolate_current_position();
        PointCloudData data(0);
        data.from_char_array(buf, size);

        for (auto & point : data.points) {
        point(0) += radar_position.x;
        point(1) += radar_position.y;
        }
        if( get_cur_state() == SYSTEM_STATE_INIT_POSE || scan_matching_pause) return;
        if(on_scan_matching_cnt_ >= 50)//////机器人初始化完成后，启动scan-matching计数
        scan_matching_cnt++;
        if ((!scan_init) || (on_scan_matching_cnt_<50)||(robot_stop_cnt<70))
        {  
            robot_stop_cnt++;////避免里程计挂掉后， 
            if ( get_cur_state() != SYSTEM_STATE_MAPPING||land_mark_mapping)//&&(!land_mark_mapping)
            {
                do_scan_matching(data, pose_scan,pos_correct);
                scan_matching_cnt = 0;
            }
            if(pos_correct && on_scan_matching_cnt_<50)
			{
			on_scan_matching_cnt_++;
			//update_last_position(pose_scan);
            printf("执行on_scan_matching_cnt_++\n");
			}
            if (on_scan_matching_cnt_ >= 50 && pos_correct)
			{/////开机scan-matching结束后，再启动位姿矫正
                if(!scan_init)
                {
                    printf("初始定位位姿为%f--%f--%f\n",pose_scan.x,pose_scan.y,pose_scan.theta);
                    scan_init = true;
                    return;
                }
                on_scan_matching_cnt_ =100;
                //pose_amcl_ = amcl_global_locate->expolate_current_position_tim(data.timestamp);
                pose_predict = expolate_current_position_tim(data.timestamp);
                //insert_index++;
                //if(insert_index>=20)
                //{update_last_position_raw(cur_pos);insert_index=0;}
                
                //update_last_position(pose_amcl_);
                //Position raw_odom = expolate_current_position_tim_raw(data.timestamp);
                //fprintf(fp_scanmatching_pos,"%ld	%d	%f	%f	%f	%f	%f	%f	%f	%f	%f\n",data.timestamp,1,pose_scan.x,pose_scan.y,pose_scan.theta,pose_amcl_.x,pose_amcl_.y,pose_amcl_.theta,raw_odom.x,raw_odom.y,raw_odom.theta);
                if(raw_pose.odom_raw_pos.size()>=10)
                {
                    int pose_select;
                    pose_scan_correct = cross_pos_correct(raw_pose,pose_select,land_mark_mapping,pose_scan);//////计算修正的数据
                    pose_scan_correct.timestamp = data.timestamp;
                    update_last_position(pose_scan_correct);
                }

            }
        }

    }
    if (channel == CHANNEL_ODOM)
    {
        Position pose;
        pose.from_char_array(buf, size);
        add_odom_data_raw(pose);
        add_odom_data(pose);
        double delta_odom =sqrt(pose.x*pose.x+pose.y*pose.y);
        total_odom += delta_odom;
        today_odom += delta_odom;
        //Position raw_odom = expolate_current_position_raw();
        //printf("   raw_odom.x=%f   ,raw_odom.y=%f,theta=%f\n",raw_odom.x,raw_odom.y,raw_odom.theta);
        //add_odom_data_robot(pose);
        //printf("---------%ld %f %f %f\n",pose.delt_t, pose.x*1000000,pose.y*1000000,pose.theta*1000000);
        if(pose.delt_t<=0)
        {if(robot_stop_cnt<65535)robot_stop_cnt++;}
        else
        robot_stop_cnt = 0;


		if(odom_list.size()>100) flush_odom_data();
		if(odom_list_raw.size()>100) flush_odom_data_raw();
        //if(odom_list_robot.size()>200) flush_odom_data_robot();
    }

    if(channel == CHANNEL_ROBOT_STS_OUT)
    {
        Warn_err_data data;
        data.from_char_array(buf, size);
        agv_sts = data.dev_sts;
    }
}
bool Core_location::start_mapping_landmark()
{
    land_mark_mapping = true;
    //amcl_global_locate->begin_high_resolution_map_create();
}
bool Core_location::stop_mapping_landmark()
{
    land_mark_mapping = false;
    //amcl_global_locate->end_high_resolution_map_create();
}
void Core_location::_begin_map_create()
{
	flush_odom_data();
	flush_odom_data_raw();
	pose2d.x = 0;
	pose2d.y = 0;
	pose2d.theta = 0;
	
    Position poseraw;
    poseraw.timestamp = get_current_time_us();
    poseraw.x = 0;
    poseraw.y = 0;
    poseraw.theta = 0;
    update_last_position_raw(poseraw);
	state = SYSTEM_STATE_MAPPING;
	printf("开始进入建图程序\n");
	scan_matching_pause = true; //先暂停跟踪定位的scan matching
    amcl_global_locate->_begin_map_create();
}
void Core_location::_end_map_create()
{
	state = SYSTEM_STATE_FREE;
	//scan_matching_pause = false; //先暂停跟踪定位的scan matching
    amcl_global_locate->_end_map_create();
}

void Core_location::start_global_locating()
{
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_INIT_POSE;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return;
    }
    pthread_mutex_unlock(&mutex_state);
    MapSharedPointer p_map;
    pose_initilized = false;
    p_map = get_current_map();
    Position pose_odom = expolate_current_position();
    //while()
    amcl_global_locate->set_init_pos(pose_odom.x,pose_odom.y,pose_odom.theta);
	amcl_global_locate->gridmap = p_map.get();

	amcl_global_locate->start_global_locating();
	pthread_create(&global_amcl_locate_thread, NULL, global_amcl_locate_thread_function, this);

    state = SYSTEM_STATE_INIT_POSE;

}
void *Core_location::global_amcl_locate_thread_function(void *param)
{
    Core_location *ptr = (Core_location *)param;
    ptr->do_amcl_global_locating();
	
    return nullptr;
}
void Core_location::do_amcl_global_locating()
{
    scan_matching_pause = true; //先暂停跟踪定位的scan matching
	sleep(1);
    while(amcl_global_locate->first_located==false)
	{
	state = SYSTEM_STATE_INIT_POSE;
    usleep(200000);
	}
    printf("agv 接收定位完成信息\n");
    Position max_pose;
    max_pose.timestamp = get_current_time_us();
    max_pose.x = amcl_global_locate->locate_pose.x;
    max_pose.y = amcl_global_locate->locate_pose.y;
    max_pose.theta = amcl_global_locate->locate_pose.theta;
    flush_odom_data();
    update_last_position(max_pose);
    pose2d = max_pose;
    flush_odom_data_raw();
    usleep(10000);
    update_last_position_raw(max_pose);
    pose2d_raw = max_pose;

    Position pose = expolate_current_position();
    printf("initial locate pose.x=%f,y=%f,theta=%f\n",pose.x,pose.y,pose.theta);
    scan_matching_pause = false;
    usleep(100000);
	//pthread_mutex_lock(&mutex_state);
    state = SYSTEM_STATE_FREE;
    //pthread_mutex_unlock(&mutex_state);
    init_locate();
	pose_initilized = true;
}
//--------------------------------------------------------------------
bool Core_location::init_locate()
{
    insert_index = 0;
    on_scan_matching_cnt_  = 0;
    scan_init = false;
    //record_pose_cnt = 0;
	//loadPoseFromserver();
    scan_matching_cnt = 0;
}
Position Core_location::do_scan_matching(PointCloudData &data, Position &pose, char *config_item)
{
    MapSharedPointer p_map;
    p_map = get_current_map();
    doing_scan_matching = true;

    float min_dist = get_configs()->get_float(config_item, "min_distance", nullptr);
    float max_dist = get_configs()->get_float(config_item, "max_distance", nullptr);

    min_dist = min_dist * min_dist;
    max_dist = max_dist * max_dist;
    int size = data.points.size();
    PointCloudData points(data.timestamp, size);

    for (int i = 0; i < size; i++)
    {
        double x = data.points[i](0);
        double y = data.points[i](1);
        double dist = x * x + y * y;
        if (dist > min_dist && dist < max_dist)
        {
            points.add_point(x, y, 1);
        }
    }

    ScanMatchingOptions opt;
    opt.occupied_space_cost_factor = get_configs()->get_float(config_item, "occupied_space_cost_factor", nullptr);
    opt.translation_delta_cost_factor = get_configs()->get_float(config_item, "translation_delta_cost_factor", nullptr);
    opt.rotation_delta_cost_factor = get_configs()->get_float(config_item, "rotation_delta_cost_factor", nullptr);
    opt.num_threads = get_configs()->get_int(config_item, "num_threads", nullptr);
    opt.max_num_iterations = get_configs()->get_int(config_item, "max_num_iterations", nullptr);
    Position optimism;
    optimism = scan_matching(pose, p_map.get(), data, opt);
    optimism.timestamp = data.timestamp;

    doing_scan_matching = false;

    return optimism;
}

void Core_location::do_scan_matching(PointCloudData &data, Position &pos_ret ,bool &is_correct)
{
    MapSharedPointer p_map;
    p_map = get_current_map();
    if (p_map.is_nullptr() || doing_scan_matching || scan_matching_pause)
    {
        // printf("Skip scan matching\n");
	is_correct = false;
        return;
    }

    float intensity_threshold = get_configs()->get_float("radar", "intensity_threshold", nullptr);
    float landmark_point_rate = get_configs()->get_float("landmark_scan_matching", "point_rate", nullptr);
    float landmark_point_angle = get_configs()->get_float("landmark_scan_matching", "point_angle", nullptr);

    // int multi_landmark_detect = get_configs()->get_int("landmark_scan_matching", "multi_landmark_detect", nullptr);
    bool land_mark_accept = false;
    bool first_init_ = false;
    int land_mark_ang_min = 0;
    int land_mark_ang_max = 0;

    int size = data.points.size();
    PointCloudData landmark_points(data.timestamp, size);
    for (int i = 0; i < size; i++)
    {
        double x = data.points[i](0);
        double y = data.points[i](1);
        if (data.intensities[i] > intensity_threshold)
        {
             if(!first_init_)
            {
                land_mark_ang_min = i;
                first_init_ = true;
            }
            landmark_points.add_point(x, y, 1);
            land_mark_ang_max = i;
        }
    }
    if(fabs(land_mark_ang_max-land_mark_ang_min)>(landmark_point_angle)) land_mark_accept=true;
    
    //Position pose = expolate_current_position_tim(data.timestamp);
    Position pose = expolate_current_position();
    //printf("cur pose for scan matching=%f  %f  %f\n",pose.x,pose.y,pose.theta);
    Position optimism = do_scan_matching(data, pose, "scan_matching");
	//printf("11111111111cur pose for scan matching=%f  %f  %f\n",pose.x,pose.y,pose.theta);
    Position optimism_mk = optimism;

    //raw_pose.odom_raw_pos.push_back(amcl_global_locate->expolate_current_position_tim_raw(data.timestamp));
    Position raw_odom = expolate_current_position_tim_raw(data.timestamp);
    raw_pose.odom_raw_pos.push_back(raw_odom);
    raw_pose.scan_match_pos.push_back(optimism);
    raw_pose.amcl_pos.push_back(amcl_global_locate->expolate_current_position_tim(data.timestamp));
    //printf("222222222222 pose for scan matching=%f  %f  %f\n",pose.x,pose.y,pose.theta);
    if (landmark_points.points.size() > size * landmark_point_rate&&land_mark_accept)
    {
        optimism_mk = do_scan_matching(landmark_points, optimism, "landmark_scan_matching");
			
	//fprintf(fp_scanmatching_pos,"%ld	%d	%f	%f	%f	%f	%f	%f	%f	%f	%f\n",optimism_mk.timestamp,2,optimism_mk.x,optimism_mk.y,optimism_mk.theta,0.0,0.0,0.0,raw_odom.x,raw_odom.y,raw_odom.theta);
	//optimism = optimism_mk;			
        raw_pose.land_mark_pos.push_back(optimism_mk);
    }
    else
        raw_pose.land_mark_pos.push_back(optimism);

    //printf("4444444444 pos_ret for scan matching=%f  %f  %f\n",pos_ret.x,pos_ret.y,pos_ret.theta);
    while(raw_pose.odom_raw_pos.size()>10) raw_pose.odom_raw_pos.erase(raw_pose.odom_raw_pos.begin());
    while(raw_pose.scan_match_pos.size()>10) raw_pose.scan_match_pos.erase(raw_pose.scan_match_pos.begin());
    while(raw_pose.amcl_pos.size()>10) raw_pose.amcl_pos.erase(raw_pose.amcl_pos.begin());
    while(raw_pose.land_mark_pos.size()>10) raw_pose.land_mark_pos.erase(raw_pose.land_mark_pos.begin());

	is_correct = true;
	pos_ret = optimism;
	//printf("777777 pos_ret for scan matching=%f  %f  %f\n",pos_ret.x,pos_ret.y,pos_ret.theta);
    //std::vector<char> sendbuf;
    //optimism.to_char_array(&sendbuf);
    //message_queue->send(CHANNEL_POSE, sendbuf.data(), sendbuf.size());
};

bool Core_location::loadPose(double xp,double yp,double theta)
{
	Position pose_odom;////从文件中读取保存的位姿
	pose_odom.timestamp = get_current_time_us();
	//double *init_position = (double*)src;
	if((fabs(xp)>fuzzy_1)||(fabs(yp)>fuzzy_1))
	{
	pose_odom.x = xp;
	pose_odom.y = yp;
	pose_odom.theta = theta;
	}
	else
	{
	pose_odom.x = pose_from_save_file.x;
	pose_odom.y = pose_from_save_file.y;
	pose_odom.theta = pose_from_save_file.theta;
	}
	printf("接收初始位姿为   %f   %f   %f\n",pose_odom.x,pose_odom.y,pose_odom.theta);
	//update_last_position(pose_odom);
    
    MapSharedPointer p_map;
    p_map = get_current_map();
    amcl_global_locate->gridmap = p_map.get();
    pose_initilized = false;
    amcl_global_locate->set_init_pos(pose_odom.x,pose_odom.y,pose_odom.theta);
    amcl_global_locate->start_global_locating();
    pthread_create(&global_amcl_locate_thread, NULL, global_amcl_locate_thread_function, this);
	
	state = SYSTEM_STATE_INIT_POSE;

	return true;
}

void Core_location::load_map_name()
{
    FILE *f;
    long len;
    char *data;
    cJSON *json,*ret;
    char   filename[128];
    char   runpath[128];

    getcwd(runpath, 127);
    sprintf(filename,"%s/map/default.json",runpath);

    if((f=fopen(filename,"rb"))==NULL)
    {  
        map_name_saved = string("default_map");  
        printf("error open default json file\n");
        return;
    }
    fseek(f,0,SEEK_END);
    len=ftell(f);
    fseek(f,0,SEEK_SET);
    data=(char*)malloc(len+1);
    fread(data,1,len,f);
    json=cJSON_Parse(data);
    fclose(f);
    if(!json)
    {
        printf("the default json file format is not correct\n");
        return ;
    }

    cJSON *name_obj  = cJSON_GetObjectItem(json,"default_map"); //
    if( NULL != name_obj )
        map_name_saved = name_obj->valuestring;
    else
        map_name_saved = string("default_map");
    //free(data);
    cJSON_Delete(json);
    printf("从文件中加载的默认地图名称   %s  \n",map_name_saved.c_str());
    return ;
}
void Core_location::set_map_name(string map_name)
{
    map_name_saved = map_name;
}
bool Core_location::loadPoseFromserver()/////lwg20201218将机器人位姿从文件中加载
{
    FILE *f;
    long len;
    char *data;
    cJSON *json,*ret;
    if((f=fopen("zero.json","rb"))==NULL)
        printf("error open zero 0 file\n");
    else if((f=fopen("zero1.json","rb"))==NULL)
    {
        printf("error open zero 1 file\n");
        return false;
    }
    fseek(f,0,SEEK_END);
    len=ftell(f);
    fseek(f,0,SEEK_SET);
    data=(char*)malloc(len+1);
    fread(data,1,len,f);
    json=cJSON_Parse(data);
    fclose(f);
    if(!json)
    {
        printf("the zero file format is not correct\n");
        //free(data);
        return false;
    }
    Position pose_odom;////从文件中读取保存的位姿
    pose_odom.timestamp = get_current_time_us();
    cJSON *zero_obj     = cJSON_GetObjectItem( json, "zero_pos");  //
    cJSON *item_sub = NULL;
    if( NULL != zero_obj ){
        item_sub = cJSON_GetObjectItem( zero_obj , "AgvX");
        if(item_sub!=NULL)
        pose_odom.x    = item_sub->valuedouble ;
        else
        pose_odom.x =0.05;

        item_sub = cJSON_GetObjectItem( zero_obj , "AgvY");
        if(item_sub!=NULL)
        pose_odom.y    = item_sub->valuedouble ;
        else
        pose_odom.y =0.05;

        item_sub = cJSON_GetObjectItem( zero_obj , "AgvTheta");
        if(item_sub!=NULL)
        pose_odom.theta    = item_sub->valuedouble ;
        else
        pose_odom.theta =0.05;
       
        item_sub = cJSON_GetObjectItem( zero_obj , "Total_odom");
        if(item_sub!=NULL)
        total_odom  = item_sub->valuedouble ;
        else
        total_odom  = 0;
    }
    else
    {
       cJSON_Delete(json);return false; 
    }
    //free(data);
    cJSON_Delete(json);
    pose_from_save_file = pose_odom;
    printf("从文件中加载的初始位姿为   %f   %f   %f\n",pose_odom.x,pose_odom.y,pose_odom.theta);
    update_last_position(pose_odom);    

    return true;
}
int Core_location::set_two_dim_data(std::list<Two_dim_data> two_in)////set the two dim list
{
    total_two_dims.erase(total_two_dims.begin(), total_two_dims.end());
    total_two_dims.clear();
    for(Two_dim_data &s_data : two_in) 
	{	
        Two_dim_data _two_dimdata(0,s_data.agv_x,s_data.agv_y,s_data.agv_theta,s_data.tag_num);
        two_dims_add(_two_dimdata);
    }

}
bool Core_location::two_dims_add(Two_dim_data &two_d)////insert the current two dim code that recognized
{
    int agvtag = two_d.tag_num;
    if (total_two_dims.empty())
        total_two_dims.push_back(two_d);
    else
    {
        for (auto iter = total_two_dims.begin();; iter++)
        {
            if (iter == total_two_dims.end())
            {
                total_two_dims.insert(iter, two_d);
                break;
            }
            if (iter->tag_num == agvtag)
            {
                *iter = two_d;
                break;
            }
        }
    }
    return false;
}
bool Core_location::Set_dm_code_correct(bool input)
{/////使能二维码矫正
   use_dm_code_correct = input;
}
/*
    use_two_dim_data_correct_robpos
    this function is used for correct the robot position,using the two dimention code
    ret 
    -1 correct false
    1 correct success
*/
int Core_location::use_two_dim_data_correct_robpos(Two_dim_data &two_d)
{
    int ret = 1;
    int agvtag = two_d.tag_num;
    if(last_dm_index == agvtag) return -2;/////
    if(!use_dm_code_correct) return -2;
    Two_dim_data two_d_saved;
    if (total_two_dims.empty())
    {    
        ret = -1;////no two dim data
        //printf("two dim dectect,but there is no two dim data loaded!!!!!!!!\n");   
    }
    else//////find the corresponding tagnum 
    {
        for (auto iter = total_two_dims.begin();; iter++)
        {
            if (iter == total_two_dims.end())
            {
                ret = -1;
                printf("the two dim data loaded,have no matching tag,please check!!!!!!!!\n");
                break;
            }
            if (iter->tag_num == agvtag)
            {
                two_d_saved = *iter;
                break;
            }
        }
    }
    if(ret == 1)
    {
        
        if(two_d.agv_x<0.03||two_d.agv_y<0.03)/////等待二维码位姿足够逼近机器人位姿后开启位姿校正
        {
        float ctheta = cos(two_d.agv_theta-M_PI);
        float stheta = sin(two_d.agv_theta-M_PI);
        float x_new = -two_d.agv_x*ctheta+two_d.agv_y*stheta+two_d_saved.agv_x;
        float y_new = -two_d.agv_x*stheta-two_d.agv_y*ctheta+two_d_saved.agv_y;
        float theta_new = two_d_saved.agv_theta + two_d.agv_theta;

        Position pos_for_correct(two_d.timestamp,x_new,y_new,theta_new);
        Position pose_predict = expolate_current_position_tim(two_d.timestamp);
        if(check_pos_near(pos_for_correct,pose_predict,0.05))
        update_last_position(pos_for_correct);
        last_dm_index = two_d.tag_num;
        printf("the two dim data correct the pos%d  %f  %f  %f\n",two_d.timestamp,x_new,y_new,theta_new);
        }
    }
    return ret;

}

void * Core_location::initial_locate_thread_function(void * param)
{
    Core_location *ptr = (Core_location *)param;

    /////load the map ,then intialize the robot postion;
    char   filename[128];
    char   runpath[128];
    ptr->load_map_name();
    getcwd(runpath, 127);
    sprintf(filename,"%s/map/%s",runpath,ptr->map_name_saved.c_str());
    printf("cur map name=%s\n",filename);
    char* data_buf;
    long file_len;
    double ini_pos[3]={0.0,0.0,0.0};
    FILE *fp;
    if((fp=fopen(filename,"rb"))==NULL)
    {
        printf("error open file\n");
    }
    else
    {
        fseek(fp, 0, SEEK_END);
        file_len = ftell(fp);
        printf("size=%ld\n",file_len);
        data_buf = (char *)malloc(file_len);
    if(file_len>10) 
    {
        fseek(fp, 0, SEEK_SET);
        int ret;
        int read_size=0;
    while (read_size<file_len) 
    {
        ret=fread(data_buf+read_size,1,file_len-read_size,fp);
        printf("read %d byte\n",ret);
        read_size+=ret;
    }
        ptr->load_grid_map(data_buf,file_len);
        
    }
    fclose(fp);
    }

    ptr->loadPoseFromserver();

    while(ptr->agv_sts.laser_disconnect == true)
    usleep(3000000);
    //
    usleep(3000000);
	ptr->loadPose(0,0,0);

    return nullptr;

}
void Core_location::load_grid_map(char *buf, int size)
{
    MapSharedPointer pointer(new MapSharedMemory(buf, size));
    pthread_mutex_lock(&mutex_map);
    p_map = pointer;
    pthread_mutex_unlock(&mutex_map);
}

void Core_location::load_grid_map(SimpleGridMap &map)
{
    MapSharedPointer pointer(new MapSharedMemory(map));
    pthread_mutex_lock(&mutex_map);
    p_map = pointer;
    pthread_mutex_unlock(&mutex_map);
}
void Core_location::set_saved_pos(Position pos_save)
{
    pose_from_save_file = pos_save;
}

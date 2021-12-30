#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <random>
#include <fstream>
#include <iostream>
#include <ceres/cubic_interpolation.h>

#include "agv.h"
#include "../common/configuration.h"
#include "../location/scan_matching.h"
#include "../common/distance_map.h"
FILE *fp_dr_pos;

//#define TEST_MODE

void AGVInstance::initialize()
{
    if (state != SYSTEM_STATE_OFFLINE)
    {
        return;
    }
    fp_dr_pos = fopen("dr_pos.txt", "w");

    //initialize mutex
    pthread_mutex_init(&mutex_state, nullptr);
    pthread_mutex_init(&mutex_dm_correct, nullptr);

    pthread_mutex_init(&mutex_radar_data, nullptr);
    pthread_mutex_init(&mutex_odom_list, nullptr);
    //initialze message queue
    message_queue = new MsgQueue(30);

#ifndef TEST_MODE

    odom = new Odometer(message_queue, CHANNEL_IMU, CHANNEL_WHEEL, CHANNEL_ODOM);
    my_runcurve = new curve_run(message_queue, CHANNEL_MOVE);
    usleep(20000);
    odom->start_mapping = true;
    state = SYSTEM_STATE_FREE;

    //initialize listener

    // message_queue->print_debug_string();
    imu = new IMUModule();
    imu->set_target(message_queue, CHANNEL_IMU);
#endif
    st = new storage();
    if (get_configs()->get_int("radar", "use_radar_type", nullptr) == 0)
    {
        double radar_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
        double radar_y = get_configs()->get_float("radar", "radar_position_y", nullptr);
        double radar_theta = get_configs()->get_float("radar", "radar_position_theta", nullptr);
        radar_position.x = radar_x;
        radar_position.y = radar_y;
        radar_position.theta = radar_theta;
    }
    else if (get_configs()->get_int("radar", "use_radar_type", nullptr) == 1)//現在的話lidar坐標系與以前的lidar坐標系不同，可能會有問題
    {
        double radar_x = get_configs()->get_float("radar", "installation_position_x", nullptr);
        double radar_y = get_configs()->get_float("radar", "installation_position_y", nullptr);
        double radar_theta = get_configs()->get_float("radar", "installation_position_theta", nullptr);
        radar_position.x = radar_x;
        radar_position.y = radar_y;
        radar_position.theta = radar_theta;
    }
    

    second_locate_max_val = get_configs()->get_float("two_dimention_code", "second_max_val", nullptr);
    if (second_locate_max_val < 0)
        second_locate_max_val = 0.5;
    record_pose_cnt = 0;
    
    laser_disconnect = true;
    imu_disconnect = true;
    motor_disconnect = true;
    lift_complete = true;

    motor = new Motor(message_queue, CHANNEL_LOGIC_OUT, CHANNEL_WHEEL, CHANNEL_MOVE);
    two_dim_ = new Two_Dim_recModule();
    control_board_ = new Control_board(message_queue, CHANNEL_LOGIC_OUT);
    logic_out_process = new Logic_process(message_queue, CHANNEL_WARN_ERR, CHANNEL_MOTOR_CTL, CHANNEL_MOVE);
    locate_modual = new Core_location(message_queue, CHANNEL_RADAR, CHANNEL_ODOM);
    obs_data_dispose = new Obs_dispose(message_queue, CHANNEL_ROBOT_STS_OUT, CHANNEL_RADAR,CHANNEL_3D_LASER_OBS_OUT);
    status_data_dispose = new Status_dispose(message_queue, CHANNEL_ROBOT_STS_OUT, CHANNEL_LASER_OBS_OUT);
    io_broad = new Io_music_modual(message_queue, CHANNEL_LOGIC_OUT);
    two_dim_->set_target(message_queue, CHANNEL_TWO_DIMENTION);
    //pallet_pose_recgnize  =  new Pallet_recgnize(message_queue,CHANNEL_RADAR);
    //radar = new RadarModule();
    //radar->set_target(message_queue, CHANNEL_RADAR);
    if (get_configs()->get_int("radar", "use_radar_type", nullptr) == 0)
    {
        radar = new RadarModule();
        radar->set_target(message_queue, CHANNEL_RADAR);
    }
    else if (get_configs()->get_int("radar", "use_radar_type", nullptr) == 1)
    {
        RadarModule3D::set_target(message_queue, CHANNEL_3D_TO_OB_RAW, CHANNEL_3D_TO_MAP_RAW); /*3D*/
        radar3d = new RadarModule3D();
        point3d = new Point_cloud_3D_data(message_queue, CHANNEL_3D_TO_MAP_RAW, CHANNEL_RADAR);
        ob_3d = new AGV_Obavoid(message_queue,CHANNEL_3D_TO_OB_RAW,CHANNEL_3D_LASER_OBS_OUT);
    }
    registe_listener();
    pthread_create(&io_control_thread, NULL, io_control_thread_function, this);
    pthread_create(&global_locate_thread, NULL, global_locate_thread_function, this);
}

#define DEL(p)        \
    if (p != nullptr) \
    {                 \
        delete p;     \
        p = nullptr;  \
    }

void AGVInstance::shutdown()
{
    if (state == SYSTEM_STATE_OFFLINE)
    {
        return;
    }
    pthread_mutex_lock(&mutex_state);

    //stop threads
    if (painting_map_thread_running)
    {
        painting_map_thread_running = false;
        pthread_join(painting_map_thread, NULL);
    }

    //free modules
    DEL(carto);
    DEL(odom);
    //DEL(devs);
    DEL(imu);
    DEL(radar);
    DEL(pallet_pose_recgnize);
    DEL(my_runcurve);
    DEL(motor);
    DEL(two_dim_);
    DEL(logic_out_process);
    DEL(control_board_);
    DEL(obs_data_dispose);
    DEL(status_data_dispose);
    //free message queue
    message_queue->clear();
    delete message_queue;

    //delete mutex
    pthread_mutex_unlock(&mutex_state);
    pthread_mutex_destroy(&mutex_state);
    pthread_mutex_destroy(&mutex_radar_data);
    pthread_mutex_destroy(&mutex_odom_list);

    pthread_mutex_destroy(&mutex_dm_correct);
    //set flag
    state = SYSTEM_STATE_OFFLINE;
}

int AGVInstance::get_system_state()
{
    pthread_mutex_lock(&mutex_state);
    int ret = state;
    pthread_mutex_unlock(&mutex_state);
    return ret;
}

//--------------------------------------------------------------------

void AGVInstance::registe_listener()
{
    message_queue->add_listener(CHANNEL_STATE, this);
    message_queue->add_listener(CHANNEL_TWO_DIMENTION, this);
    message_queue->add_listener(CHANNEL_RADAR, this);
    message_queue->add_listener(CHANNEL_WARN_ERR, this);
    message_queue->add_listener(CHANNEL_ARRIVE, this);
}

void AGVInstance::recieve_message(const int channel, char *buf, const int size) //例行的,不受状态变化影响
{
    if (channel == CHANNEL_ARRIVE)
    {
        int cmd = *((int *)buf);
        if ((cmd == lift_up_done) || (cmd == lift_turn_done))
            lift_complete = true;
        else if (cmd == traj_done)
        {
            if (state == SYSTEM_STATE_NAVIGATING)
                //if(!my_runcurve->busying)
                state = SYSTEM_STATE_FREE;
            printf("agv main receive trajectory complete cmd\n");
        }
    }

    if (channel == CHANNEL_WARN_ERR)
    {
        Warn_err_data errdata;
        errdata.from_char_array(buf, size);
        warning_code = errdata.warn_code;
        error_code = errdata.err_code;
        st_device_sts device_sts = errdata.dev_sts;
        collision_level = device_sts.collision_level;
    }
    if (channel == CHANNEL_STATE)
    {
        Position pose;
        pose.from_char_array(buf, size);

        int arg_int1 = *((int *)buf);
        int arg_int2 = *((int *)(buf + 4));
        int arg_int3 = *((int *)(buf + 8));
        int arg_int4 = *((int *)(buf + 12));
        int arg_int5 = *((int *)(buf + 16));
        //		printf("###mcu state.输入=%d,输出=%d,电量=%d,速度=%d,角速度=%d\n",arg_int1,arg_int2,arg_int3,arg_int4,arg_int5);
        mcu_input = arg_int1;
        mcu_output = arg_int2;
        mcu_battery_level = arg_int3;
        mcu_speed = arg_int4;
        mcu_omega = arg_int5;
    }
    if (channel == CHANNEL_TWO_DIMENTION) ////excute the two dimention locate
    {
        Two_dim_data dim_data;
        dim_data.from_char_array(buf, size);
        if (dim_data.device_send == TWO_CODE1) ////if received the locating two dimention code
        {
            cur_tagnum_locate = dim_data.tag_num;
            if (dm_code_record) ////if mapping the two dimention
            {
                //////change the two dim pose into the map pos
                float ctheta = cos(dim_data.agv_theta - M_PI);
                float stheta = sin(dim_data.agv_theta - M_PI);
                pallet_bias_pos.x = ctheta * (-dim_data.agv_x) - stheta * (-dim_data.agv_y);
                pallet_bias_pos.y = stheta * (-dim_data.agv_x) + ctheta * (-dim_data.agv_y);
                pallet_bias_pos.theta = dim_data.agv_theta;
                printf("标记的二维码坐标数据为x=%f,y=%f,theta=%f\n", dim_data.agv_x, dim_data.agv_y, dim_data.agv_theta);
                Position rpose = expolate_current_position();
                Position corect_pose = rpose * pallet_bias_pos;
                //////modify the dim_data pose to the map pos
                dim_data.agv_x = corect_pose.x;
                dim_data.agv_y = corect_pose.y;
                dim_data.agv_theta = corect_pose.theta;
                two_dims_add(dim_data); /////save the two dim data to the list
            }
        }
        else if (dim_data.device_send == TWO_CODE2) ////if received the pallet two dimention code
        {
            cur_tagnum_pallet = dim_data.tag_num;
            pallet_bias_pos.x = dim_data.agv_x;
            pallet_bias_pos.y = dim_data.agv_y;
            pallet_bias_pos.theta = dim_data.agv_theta;
            pthread_mutex_lock(&mutex_dm_correct);
            if (b_two_dim_correct && (!my_runcurve->busying)) ////if use the two dim to correct the pallet, then call the two dim correct function.
            {

                if ((fabs(pallet_bias_pos.y) > second_locate_max_val) || (fabs(pallet_bias_pos.x) > second_locate_max_val) || (fabs(pallet_bias_pos.theta) > second_locate_max_val))
                {
                    printf("二维码识别位姿误差超范围x=%f,y=%f,theta=%f\n", pallet_bias_pos.x, pallet_bias_pos.y, pallet_bias_pos.theta);
                    my_runcurve->task_complete_send();
                    device_sts.dm_code_error = true;
                    b_two_dim_correct = false;
                }
                else if (dim_data.state > 0 && ((fabs(pallet_bias_pos.y) > 0.015) || (fabs(pallet_bias_pos.x) > 0.015) || (fabs(pallet_bias_pos.theta) > 0.05)))
                {

                    use_two_dim_correct_pallet(pallet_bias_pos);
                    b_two_dim_correct = false;

                    /*if (use_two_dim_correct_pallet(pallet_bias_pos)==false){
                my_runcurve->task_complete_send();
                device_sts.dm_code_error =  true;
                }
               b_two_dim_correct = false; */
                }
                else
                {
                    if (b_two_dim_correct)
                    {
                        printf("4二维码识别位姿误差x=%f,y=%f\n", pallet_bias_pos.x, pallet_bias_pos.y);
                        my_runcurve->lift_task_run();
                        b_two_dim_correct = false;
                    }
                }
            }
            pthread_mutex_unlock(&mutex_dm_correct);
        }
    }
    if (channel == CHANNEL_RADAR)
    {
        ///////////////////////首先判断机器人的运行距离，这里先写死距离方向0.01m触发和角度方向0.1弧度触发
        record_pose_cnt++;
        if (record_pose_cnt > 20)
        {
            savePoseToServer();
            record_pose_cnt = 0;
        }
        Position pose_scan;
        Position pose_amcl_;
        Position pose_scan_correct;
        bool pos_correct = false;
        PointCloudData data(0);
        data.from_char_array(buf, size);
        pthread_mutex_lock(&mutex_radar_data);
        for (auto &point : data.points)
        {
            point(0) += radar_position.x;
        }
        pthread_mutex_unlock(&mutex_radar_data);
        last_radar_data = data;

        if (get_system_state() == SYSTEM_STATE_MAPPING_LANDMARK && (land_mark_mapping))
            handle_landmark_data_on_painting(data);
    }
}
void AGVInstance::handle_landmark_data_on_painting(PointCloudData &data)
{
    float intensity_threshold = get_configs()->get_float("radar", "intensity_threshold", nullptr);
    float landmark_point_rate = get_configs()->get_float("landmark_scan_matching", "point_rate", nullptr);
    float landmark_point_angle = get_configs()->get_float("landmark_scan_matching", "point_angle", nullptr);

    LandmarkRadarData tmp;
    Position pose = expolate_current_position();
    tmp.pose[0] = pose.x;
    tmp.pose[1] = pose.y;
    tmp.pose[2] = pose.theta;
    int size = data.points.size();
    bool land_mark_accept = false;
    bool first_init_ = false;
    int land_mark_ang_min = 0;
    int land_mark_ang_max = 0;
    for (int i = 0; i < size; i++)
    {
        if (data.intensities[i] > intensity_threshold)
        {
            if (!first_init_)
            {
                land_mark_ang_min = i;
                first_init_ = true;
            }
            double point;
            float *p_float = (float *)(&point);
            p_float[0] = data.points[i](0);
            p_float[1] = data.points[i](1);
            tmp.points.push_back(point);
            land_mark_ang_max = i;
        }
    }
    if (fabs(land_mark_ang_max - land_mark_ang_min) > (landmark_point_angle))
        land_mark_accept = true;
    if (tmp.points.size() > (int)(size * landmark_point_rate) && land_mark_accept)
    {
        landmark_radar_datas.push_back(tmp);
    }
}

Position AGVInstance::expolate_current_position()
{
    Position ret = locate_modual->expolate_current_position(); //////////XXXXXXXXXXXXXGGGGGG
    return ret;
}
Position AGVInstance::expolate_current_position_raw()
{
    Position ret = locate_modual->expolate_current_position_raw(); //////////XXXXXXXXXXXXXGGGGGG
    return ret;
}
Position AGVInstance::expolate_current_position_tim(long timestamp)
{
    Position ret = locate_modual->expolate_current_position_tim(timestamp); //////////XXXXXXXXXXXXXGGGGGG
    return ret;
}
Position AGVInstance::expolate_current_position_tim_raw(long timestamp)
{
    Position ret = locate_modual->expolate_current_position_tim_raw(timestamp); //////////XXXXXXXXXXXXXGGGGGG
    return ret;
}
void AGVInstance::load_grid_map(SimpleGridMap &map)
{
    locate_modual->load_grid_map(map);
}

void AGVInstance::load_grid_map(char *buf, int size)
{
    locate_modual->load_grid_map(buf, size);
}

//--------------------------------------------------------------------

PointCloudData AGVInstance::get_last_radar_data()
{
    PointCloudData ret;
    //pthread_mutex_lock(&mutex_radar_data);
    ret = last_radar_data;
    //pthread_mutex_unlock(&mutex_radar_data);
    return ret;
}

//--------------------------------------------------------------------
bool AGVInstance::start_navigating(CapRoute *r) //todo 测试通过后删除此入口
{
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_NAVIGATING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);

    //state = SYSTEM_STATE_NAVIGATING;
    NavModuleOption opt;
    opt.msg_queue = message_queue;
    opt.pose_channel = CHANNEL_POSE;
    opt.arrive_channel = CHANNEL_ARRIVE;
    opt.route = r;
    nav = new NavModule(opt);
}
void AGVInstance::ChangeAgv2Nav() /////当接收到转换到nav模式
{
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_NAVIGATING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return;
    }
    lift_complete = false;
    pthread_mutex_unlock(&mutex_state);
}
bool AGVInstance::start_navigating(RouteChain c)
{
    stop_navigating();
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_NAVIGATING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
    //state = SYSTEM_STATE_NAVIGATING;
    NavModuleOption opt;
    opt.msg_queue = message_queue;
    opt.pose_channel = CHANNEL_POSE;
    opt.arrive_channel = CHANNEL_ARRIVE;
    opt.chain = c;
    nav = new NavModule(opt);
}

bool AGVInstance::stop_navigating()
{

    pthread_mutex_lock(&mutex_state);
    nav_thread_running = false;
    if (state == SYSTEM_STATE_NAVIGATING)
    {
        state = SYSTEM_STATE_FREE;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
    if (nav != nullptr)
    {
        DEL(nav);
        nav = nullptr;
    }
    //exec_route=0xffff;
    cancel_path();
    exec_route &= ~(1 << 16);
    exec_cap = 0xfffe;
    return true;
}
double AGVInstance::obtain_imu_angle()
{
    double ret_ang = odom->obtain_imu_angle();
    return ret_ang;
}

bool AGVInstance::start_mapping()
{
    if (laser_disconnect == true || imu_disconnect || motor_disconnect)
        return false;
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_MAPPING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
    DEL(carto);
    char carto_dir[200];
    get_configs()->get_string("carto", "config_files_dir", carto_dir, nullptr);
    char carto_name[200];
    get_configs()->get_string("carto", "config_file_name", carto_name, nullptr);
    printf("config_files_dir=%s,config_file_name=%s\n", carto_dir, carto_name);
    CartoModuleOption opt(carto_dir, carto_name);
    opt.radar_scan_time = get_configs()->get_float("carto", "radar_scan_time", nullptr);
    opt.pose_mathching_score = get_configs()->get_float("carto", "pose_matching_score", nullptr);
    opt.msg_queue = message_queue;
    opt.pose_channel = CHANNEL_POSE;
    opt.imu_channel = CHANNEL_IMU;
    //opt.imu_channel = CHANNEL_ODOM_IMU;
    opt.radar_channel = CHANNEL_RADAR;
    carto = new CartoModule(opt);

    landmark_radar_datas.clear();

    odom->start_mapping = true;
    locate_modual->_begin_map_create();
    painting_map_thread_running = true;
    pthread_create(&painting_map_thread, nullptr, painting_map_thread_function, this);

    return true;
};
bool AGVInstance::stop_mapping()
{
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_MAPPING)
    {
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
    locate_modual->_end_map_create();
    painting_map_thread_running = false;
    return true;
}
bool AGVInstance::start_two_dims_create()
{
    ////first clear the dimetions that have saved
    total_two_dims.erase(total_two_dims.begin(), total_two_dims.end());
    total_two_dims.clear();

    return true;
}
bool AGVInstance::end_two_dims_create()
{
    if (total_two_dims.empty())
        return false; ////no two dim finded
    cJSON *root = NULL;
    cJSON *TWDIM = NULL;
    cJSON *array = NULL;
    cJSON *addobj = NULL;
    char *jsonout = NULL;
    root = cJSON_CreateObject(); ////////创建根节点
    cJSON_AddItemToObject(root, "two_dim", TWDIM = cJSON_CreateArray());
    for (auto it = total_two_dims.begin();; ++it)
    {
        if (it == total_two_dims.end())
            break;
        cJSON_AddItemToArray(TWDIM, addobj = cJSON_CreateObject());
        cJSON_AddNumberToObject(addobj, "AgvX", it->agv_x);
        cJSON_AddNumberToObject(addobj, "AgvY", it->agv_y);
        cJSON_AddNumberToObject(addobj, "AgvTheta", it->agv_theta);
        cJSON_AddNumberToObject(addobj, "TagNum", it->tag_num);
        printf("write the two dim x%f y%f theta%f tag%d into file 2\n", it->agv_x, it->agv_y, it->agv_theta, it->tag_num);
    }
    jsonout = cJSON_Print(root);
    //打开一个info.json文件，并写入json内容
    FILE *fp = fopen("TWO_DIM.json", "w");
    fwrite(jsonout, strlen(jsonout), 1, fp);
    fclose(fp); //关闭文件
    cJSON_Delete(root);
    free(jsonout);
    locate_modual->set_two_dim_data(total_two_dims);
    return true;
}
bool AGVInstance::load_two_dims_file()
{
    FILE *f;
    long len;
    char *data;
    cJSON *json, *ret;
    if ((f = fopen("TWO_DIM.json", "rb")) == NULL)
    {
        printf("error open TWO_DIM file\n");
        return false;
    }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    data = (char *)malloc(len + 1);
    fread(data, 1, len, f);
    json = cJSON_Parse(data);
    fclose(f);
    if (!json)
    {
        printf("the TWO_DIM file format is not correct\n");
        //free(data);
        return false;
    }
    cJSON *dim_arry = cJSON_GetObjectItem(json, "two_dim"); //
    if (NULL != dim_arry)
    {
        cJSON *two_dim_list = dim_arry->child;
        while (two_dim_list != NULL)
        {
            float p_x = cJSON_GetObjectItem(two_dim_list, "AgvX")->valuedouble;
            float p_y = cJSON_GetObjectItem(two_dim_list, "AgvY")->valuedouble;
            float p_theta = cJSON_GetObjectItem(two_dim_list, "AgvTheta")->valuedouble;
            int p_num = cJSON_GetObjectItem(two_dim_list, "TagNum")->valueint;
            printf("the paralized two dim x=%f y=%f theta=%f tag=%d\n", p_x, p_y, p_theta, p_num);
            Two_dim_data _two_dimdata(0, p_x, p_y, p_theta, p_num);
            two_dims_add(_two_dimdata);
            two_dim_list = two_dim_list->next;
        }
    }
    //free(data);
    cJSON_Delete(json);
    locate_modual->set_two_dim_data(total_two_dims);
    return true;
}
bool AGVInstance::two_dims_add(Two_dim_data &two_d) ////insert the current two dim code that recognized
{

    int agvtag = two_d.tag_num;
    if (agvtag <= 0)
    {
        printf("the record tagnum=%d\n", agvtag);
        dm_code_record = false;
        return false;
    }
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
    if (dm_code_record)
    {
        dm_code_record = false;
        end_two_dims_create();
    }
    return true;
}
bool AGVInstance::start_mapping_landmark()
{
    if (laser_disconnect == true || imu_disconnect)
        return false;
    if (get_current_map().is_nullptr())
        return false;
    printf("1111111111111111111111111111111111111111111111111111111111111111\n");
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE && locate_modual->pose_initilized)
    {
        state = SYSTEM_STATE_MAPPING_LANDMARK;
        land_mark_mapping = true;
        locate_modual->start_mapping_landmark();
        obs_data_dispose->begin_high_resolution_map_create();
        start_two_dims_create();
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
    landmark_radar_datas.clear();
    return true;
};
bool AGVInstance::stop_mapping_landmark()
{
    //if(laser_disconnect == true || imu_disconnect || motor_disconnect) return false;
    if (laser_disconnect == true || imu_disconnect)
        return false;
    if (get_current_map().is_nullptr())
        return false;
    if (state != SYSTEM_STATE_MAPPING_LANDMARK)
        return false;
    if (!land_mark_mapping)
        return false;

    long start_time = get_current_time_us();
    long opt_time;
    MapSharedPointer p_map;
    p_map = get_current_map();
    LandmarkMapGenerator gen(get_configs()->get_float("map", "landmark_map_resolution", nullptr));
    int frame_count = 0;
    int point_count = 0;
    for (LandmarkRadarData &data : landmark_radar_datas)
    {
        Position pose(1, data.pose[0], data.pose[1], data.pose[2]);
        PointCloudData radardata(1);
        for (double d : data.points)
        {
            double farray = d;
            float *p_float = (float *)(&farray);
            radardata.add_point(p_float[0], p_float[1]);
        }
        ScanMatchingOptions opt;
        opt.occupied_space_cost_factor = get_configs()->get_float("scan_matching", "occupied_space_cost_factor", nullptr);
        opt.translation_delta_cost_factor = get_configs()->get_float("scan_matching", "translation_delta_cost_factor", nullptr);
        opt.rotation_delta_cost_factor = get_configs()->get_float("scan_matching", "rotation_delta_cost_factor", nullptr);
        opt.num_threads = get_configs()->get_int("scan_matching", "num_threads", nullptr);
        opt.max_num_iterations = get_configs()->get_int("scan_matching", "max_num_iterations", nullptr);

        Position optimism = scan_matching(pose, p_map.get(), radardata, opt);
        for (auto &point : radardata.points)
        {
            Position p(1, point(0), point(1), 0);
            Position after_trans = optimism * p;
            gen.add_point(after_trans.x, after_trans.y);
            point_count++;
            //fprintf(fp_t,"%ld,%ld\n",after_trans.x, after_trans.y);
        }
        frame_count++;
    }
    p_map->landmark = gen.to_landmark_map();
    printf("Optimism done, draw %d landmark frames\n", frame_count);
    opt_time = (get_current_time_us() - start_time) / 1000;
    printf("Using time %lds,%ldms\n", opt_time / 1000, opt_time % 1000);
    usleep(500000);
    state = SYSTEM_STATE_FREE;
    land_mark_mapping = false;
    locate_modual->stop_mapping_landmark();
    //printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
    obs_data_dispose->end_high_resolution_map_create();
    return true;
};
//

void *AGVInstance::painting_map_thread_function(void *param) //定时从cartographer提取地图,反光板优化
{
    //FILE *fp_t;
    //fp_t = fopen("refletc_points.txt","w");
    AGVInstance *ptr = (AGVInstance *)param;
    long time_us = get_configs()->get_long("carto", "paint_map_period_us", nullptr);
    long start_time = get_current_time_us();
    long opt_time;
    usleep(time_us);
    MapSharedPointer p_map;
    usleep(3000000);
    while (ptr->painting_map_thread_running)
    {
        std::vector<char> map_data;
        ptr->carto->paint_map(&map_data);
        ptr->load_grid_map(map_data.data(), map_data.size());
        usleep(time_us);
    }
    ptr->state = SYSTEM_STATE_OPTIMISM;
    start_time = get_current_time_us();
    ptr->carto->stop_and_optimize();
    opt_time = (get_current_time_us() - start_time) / 1000;
    printf("Using time %lds,%ldms\n", opt_time / 1000, opt_time % 1000);
    if (ptr->carto)
    {
        //printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
        std::vector<char> map_data;
        ptr->carto->paint_map(&map_data);
        ptr->load_grid_map(map_data.data(), map_data.size());
        usleep(10000);
        //printf("cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
    }
    DEL(ptr->carto);
    printf("Optimism landmarks\n");
    start_time = get_current_time_us();

    p_map = ptr->get_current_map();
    //printf("dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd\n");
    //clm,保存地图至本地,从停止建图移至此处
    /*	printf("###id=%d,name=%s\n",get_global_agv_instance()->map_id,get_global_agv_instance()->map_name);
	if((get_global_agv_instance()->map_id<=40)&&(get_global_agv_instance()->map_id>=1)&&(get_global_agv_instance()->map_name[0]!=0)){
		std::vector<char> map_data;
		//保存地图
//		get_global_agv_instance()->latest_map->get_data(&map_data);
		p_map->to_char_array(&map_data);
		int size = map_data.size();
		printf("###size=%d\n",size);
		char *p = map_data.data();
		FILE* fp;
		if((fp=fopen(get_global_agv_instance()->map_name,"wb"))==NULL){
			printf("error open file\n");
		}else{
			
			int ret;
			int write_len=0;
			while(write_len<size){
				ret=fwrite(p+write_len,1,size-write_len,fp);
				write_len+=ret;
			}
			fclose(fp);
			get_global_storage()->modify_map_slot(get_global_agv_instance()->map_id,get_global_agv_instance()->map_name);
			memset(get_global_agv_instance()->map_name,0,1);
			get_global_agv_instance()->map_id=0;
		}
	}
*/

    ptr->state = SYSTEM_STATE_FREE;
    usleep(100000);
    //printf("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\n");
    ptr->start_global_locating();
    //printf("fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
    return nullptr;
}

void AGVInstance::start_global_locating()
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
    locate_modual->start_global_locating();
    usleep(20000);
    pthread_create(&global_locate_thread, NULL, global_locate_thread_function, this);
}
void *AGVInstance::global_locate_thread_function(void *param)
{
    sleep(1);
    AGVInstance *ptr = (AGVInstance *)param;
    ptr->do_global_locating();

    return nullptr;
}
void AGVInstance::do_global_locating()
{
    while (locate_modual->pose_initilized == false)
    {
        usleep(200000);
        state = SYSTEM_STATE_INIT_POSE;
    }
    Position pose = expolate_current_position();
    printf("ended locate pose.x=%f,y=%f,theta=%f\n", pose.x, pose.y, pose.theta);

    state = SYSTEM_STATE_FREE;
}

//1.25,0.85,150,20
int AGVInstance::pallet_recognize(float len, float width, float angle_sech, int clc_cnt, Position &result_pose) ////托盘位姿识别
{
    int rec_sult;
    rec_sult = pallet_pose_recgnize->pallet_recognize(len, width, angle_sech, clc_cnt, result_pose);
    return rec_sult;
}
//1.25,0.85,180,20
int AGVInstance::legs_recognize(float len, float width, float angle_sech, int clc_cnt, Position &leg1_pose, Position &leg2_pose) ////托盘退部识别
{
    int rec_sult;
    rec_sult = pallet_pose_recgnize->legs_recognize(len, width, angle_sech, clc_cnt, leg1_pose, leg2_pose);
    return rec_sult;
}
bool AGVInstance::use_two_dim_correct_pallet(Position delt_pos)
{
    if (!b_two_dim_correct)
        return false;
    printf("执行二维码位姿矫正部分代码  %f  %f   %f  \n", delt_pos.x, delt_pos.y, delt_pos.theta);
    //if(delt_pos.theta>(0.5*M_PI)) delt_pos.theta -= M_PI;
    //if(delt_pos.theta<(-0.5*M_PI)) delt_pos.theta += M_PI;
    collision_switch = false;
    lift_up_turn_switch = false;
    vector<Position> caps;
    vector<float> speed;
    vector<char> dir;
    vector<char> occll;
    vector<char> inv_cps;
    vector<char> do_acts;
    vector<float> fix_angs;
    float ctheta = cos(delt_pos.theta - 0.5 * M_PI);
    float stheta = sin(delt_pos.theta - 0.5 * M_PI);
    Position recal_pos;
    recal_pos.x = (ctheta * delt_pos.x - stheta * delt_pos.y);
    recal_pos.y = (stheta * delt_pos.x + ctheta * delt_pos.y);
    recal_pos.theta = -delt_pos.theta;

    Position rpose = expolate_current_position();
    Position bkdel = Position(0, -0.27, recal_pos.y, 0);
    Position bpose = rpose * bkdel;

    caps.push_back(rpose);
    speed.push_back(0.2f);
    dir.push_back(1);
    occll.push_back(0);
    inv_cps.push_back(0);
    do_acts.push_back(0);
    fix_angs.push_back(0);

    double theta = atan2(bpose.y - rpose.y, bpose.x - rpose.x);
    theta -= M_PI;

    if (recal_pos.theta > (0.5 * M_PI))
        recal_pos.theta -= M_PI;
    if (recal_pos.theta < (-0.5 * M_PI))
        recal_pos.theta += M_PI;
    Position corect_pose = rpose * recal_pos;
    caps.push_back(Position(1, bpose.x, bpose.y, theta));
    speed.push_back(0.2f);
    dir.push_back(1);
    occll.push_back(0);
    inv_cps.push_back(0);
    do_acts.push_back(0);
    fix_angs.push_back(0);
    //theta=atan2(corect_pose.y - rpose.y,corect_pose.x - rpose.x);
    caps.push_back(Position(2, corect_pose.x, corect_pose.y, corect_pose.theta));
    speed.push_back(0.2f);
    dir.push_back(0);
    occll.push_back(0);
    inv_cps.push_back(0);
    do_acts.push_back(LIFT_UP_CMD);
    fix_angs.push_back(0);

    my_runcurve->initial_path(caps, speed, dir, 1, occll, inv_cps, do_acts, fix_angs);
    //start_path(caps,speed,dir,1);
    //b_two_dim_correct = false;
    return true;
}
int AGVInstance::start_path(vector<Position> pathpoints, vector<float> vel_points, vector<char> inverset, char doact, vector<char> occi, vector<char> inv_cps, vector<char> acts, vector<float> angles)
////ret -1 failed; 1 success;doact(0-no action,1-up,2-down,3-twodim up)
{
    if (control_board_->charging_flag > 0)
        return -1; ////如果系统正在充电，则直接返回
    pthread_mutex_lock(&mutex_state);
    if ((state == SYSTEM_STATE_FREE) || (state == SYSTEM_STATE_NAVIGATING))
    {
        state = SYSTEM_STATE_NAVIGATING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return -4; ///////if the state is not correct,return -4
    }
    pthread_mutex_unlock(&mutex_state);

    //if(laser_disconnect == true || imu_disconnect ) return -3;///if radar is not connected , return -2;
    if (locate_modual->pose_initilized == false)
        return -2; /////if robot pos is not initialized , return -2;
    //if(AUTO_MODE != board_sts._manual_auto)///if robot pos is not in auto mode , return -3;
    int device_enable = get_configs()->get_int("two_dimention_code", "second_enable", nullptr);
    int rec_sult;
    if (doact == 1 || doact == 2)
        lift_complete = false;

    if (device_enable > 0 && (!two_dim_->device2_disconnect)) //////托盘二维码使能且连接成功
    {
        if (doact == 1)
        {
            rec_sult = my_runcurve->initial_path(pathpoints, vel_points, inverset, LIFT_2D_UP_CMD, occi, inv_cps, acts, angles);
            b_two_dim_correct = true;
        }
        else
            rec_sult = my_runcurve->initial_path(pathpoints, vel_points, inverset, doact, occi, inv_cps, acts, angles);
    }
    else
        rec_sult = my_runcurve->initial_path(pathpoints, vel_points, inverset, doact, occi, inv_cps, acts, angles);

    if (rec_sult == -1)
        stop_navigating();
    return rec_sult;
}

bool AGVInstance::cancel_path()
{
    bool rec_sult;
    rec_sult = my_runcurve->cancel_path();
    return rec_sult;
}
bool AGVInstance::stop_path()
{
    bool rec_sult;
    rec_sult = my_runcurve->stop_path();
    return rec_sult;
}
bool AGVInstance::continue_path()
{
    bool rec_sult;
    rec_sult = my_runcurve->continue_path();
    return rec_sult;
}
bool AGVInstance::reset_motor()
{
    bool rec_sult;
    rec_sult = motor->reset_motor();
    return rec_sult;
}
bool AGVInstance::loadPoseFromserver() /////lwg20201218将机器人位姿从文件中加载
{
    bool rec_sult;
    rec_sult = locate_modual->loadPoseFromserver();
    return rec_sult;
}

bool AGVInstance::loadPose(double xp, double yp, double theta)
{
    bool rec_sult;
    rec_sult = locate_modual->loadPose(xp, yp, theta);
    pthread_create(&global_locate_thread, NULL, global_locate_thread_function, this);
    return rec_sult;
}
void *AGVInstance::io_control_thread_function(void *param)
{
    AGVInstance *ptr = (AGVInstance *)param;
    sleep(1);
    ptr->load_two_dims_file();
    int out_cnt = 0;
    while (true)
    {
        out_cnt++;
        ptr->update_agv_status();
        if (out_cnt > 900)
        {
            printf("radar_disconnect=%d,radar_error=%d,lift_complete=%d\n", ptr->laser_disconnect, ptr->laser_error, ptr->lift_complete);
            printf("motor_disconnect=%d,motor_error=%d\n", ptr->motor_disconnect, ptr->motor_error);
            printf("imu_disconnect=%d,board_disconnect=%d\n", ptr->imu_disconnect, ptr->board_disconnect);
            printf("warning_code=%d,error_code=%d,i_step=%d\n", ptr->warning_code, ptr->error_code, ptr->my_runcurve->i_step);
            out_cnt = 0;
        }
        usleep(25000);
    }

    return nullptr;
}
void AGVInstance::update_agv_status()
{

    board_sts.obs_avoid_f = control_board_->obs_avoid_f; //////避障传感器
    board_sts.obs_avoid_b = control_board_->obs_avoid_b;
    board_sts.obs_avoid_l = control_board_->obs_avoid_l;
    board_sts.obs_avoid_r = control_board_->obs_avoid_r;

    board_sts.anti_colli_f = control_board_->anti_colli_f; //////防撞条
    board_sts.anti_colli_b = control_board_->anti_colli_b;
    board_sts.anti_colli_l = control_board_->anti_colli_l;
    board_sts.anti_colli_r = control_board_->anti_colli_r;

    board_sts._stop = control_board_->_stop;       //////暂停
    board_sts._continu = control_board_->_continu; //////暂停
    //board_sts._reset = control_board_->_reset;//////复位
    //board_sts._manual_auto = control_board_->_manual_auto;//////手动-0 ，自动-1
    board_sts._battary = control_board_->_battary; ////电池电量
    //board_sts._charging = control_board_->charging_flag;

    // laser_error = radar->radar_error;
    // laser_disconnect = radar->radar_disconnect;
    if(get_configs()->get_int("radar","use_radar_type",nullptr) == 0){
	    laser_error = radar->radar_error;
        laser_disconnect = radar->radar_disconnect;
    }
    else if(get_configs()->get_int("radar","use_radar_type",nullptr) == 1){
		laser_error = radar3d->radar_error;
        laser_disconnect = radar3d->radar_disconnect;
    }

    motor_error = motor->motor_error;
    motor_disconnect = motor->motor_disconnect;
    imu_disconnect = imu->imu_disconnect;
    board_disconnect = control_board_->board_disconnect;
    _battary_warning = control_board_->_battary_warning; ////电量报警

    device_sts.laser_error = laser_error;
    device_sts.laser_disconnect = laser_disconnect;
    device_sts.motor_error = motor_error;
    device_sts.motor_disconnect = motor_disconnect;
    device_sts.imu_disconnect = imu_disconnect;
    device_sts.board_disconnect = board_disconnect;
    device_sts._battary_warning = _battary_warning; ////电量报警
    device_sts.pose_initilized = locate_modual->pose_initilized;
    device_sts.collision_switch = collision_switch;
    device_sts.device2_disconnect = (two_dim_->device2_disconnect | two_dim_->td_code_2_error);
    device_sts.device1_disconnect = (two_dim_->device1_disconnect | two_dim_->td_code_1_error);
    device_sts.lift_up_flag = motor->lft_drv.lift_up_flag;
    device_sts.lift_up_turn_switch = lift_up_turn_switch;
    device_sts.charging = control_board_->charging_flag;

    ob_3d_sts.ahead_obstacle = obs_data_dispose->radar_3d_sts.ahead_obstacle;
    ob_3d_sts.rear_obstacle = obs_data_dispose->radar_3d_sts.rear_obstacle;
    ob_3d_sts.left_obstacle = obs_data_dispose->radar_3d_sts.left_obstacle;
    ob_3d_sts.right_obstacle = obs_data_dispose->radar_3d_sts.right_obstacle;


    if (board_sts._manual_auto != control_board_->_manual_auto) ////切换手动自动后，取消正在执行的任务
    {
        if (board_sts._manual_auto = 0)
            cancel_path();
        else if (board_sts._manual_auto = 1)
            continue_path();
        board_sts._manual_auto = control_board_->_manual_auto;
    }

    if (board_sts._emgcy != control_board_->_emgcy) /////急停状态切换后，取消正在执行的任务
    {
        stop_path();
        board_sts._emgcy = control_board_->_emgcy;
    }
    if (board_sts._stop)
    {
        stop_path();
        control_board_->_stop = false;
    }
    if (board_sts._continu)
    {
        continue_path();
        control_board_->_continu = false;
    }

    if (board_sts._reset != control_board_->_reset)
    {
        reset_motor();
        board_sts._reset = control_board_->_reset;
        device_sts.dm_code_error = false;
        continue_path();
    }

    //if (state == SYSTEM_STATE_NAVIGATING)
    //if(!my_runcurve->busying)
    //{
    //state = SYSTEM_STATE_FREE;
    //printf("agv main receive trajectory complete cmd\n");
    //}
    std::vector<char> sendbuf;
    Warn_err_data data(0, 0);
    data.ctl_sts = board_sts;
    data.dev_sts = device_sts;
    data.ob_3d_sts = ob_3d_sts;
    data.to_char_array(&sendbuf);
    message_queue->send(CHANNEL_ROBOT_STS_OUT, sendbuf.data(), sendbuf.size());
}
//void AGVInstance::do_scan_matching(PointCloudData &data)

bool AGVInstance::savePoseToServer() /////lwg20201218将机器人实时位姿写入到文件
{
    if (get_system_state() == SYSTEM_STATE_MAPPING)
        return false; //////机器人建图过程中不记录位姿数据
                      //if((!device_sts.pose_initilized)) return false;/////机器人初始化未完成，不记录位姿数据
    if (!locate_modual->pose_initilized)
        return false; /////机器人初始化未完成，不记录位姿数据
    cJSON *root = NULL;
    cJSON *ZEROPOS = NULL;
    char *jsonout = NULL;
    Position pose_odom;
    pose_odom = expolate_current_position(); ///////获取当前时刻机器人的位姿数据
    locate_modual->set_saved_pos(pose_odom);
    root = cJSON_CreateObject(); ////////创建根节点
    cJSON_AddItemToObject(root, "zero_pos", ZEROPOS = cJSON_CreateObject());
    cJSON_AddNumberToObject(ZEROPOS, "AgvX", pose_odom.x);
    cJSON_AddNumberToObject(ZEROPOS, "AgvY", pose_odom.y);
    cJSON_AddNumberToObject(ZEROPOS, "AgvTheta", pose_odom.theta);
    cJSON_AddNumberToObject(ZEROPOS, "Total_odom", locate_modual->total_odom);
    jsonout = cJSON_Print(root);
    //打开一个info.json文件，并写入json内容
    FILE *fp;
    if (recor_file_index == 1)
    {
        fp = fopen("zero.json", "w");
        recor_file_index++;
    }
    else
    {
        fp = fopen("zero1.json", "w");
        recor_file_index = 1;
    }
    fwrite(jsonout, strlen(jsonout), 1, fp);
    fclose(fp); //关闭文件
    cJSON_Delete(root);
    free(jsonout);

    /*	Position pose_odom;
	pose_odom = expolate_current_position();///////获取当前时刻机器人的位姿数据
	pose_from_save_file = pose_odom;
	FILE *IniFile;
	IniFile = fopen("zero.ini", "w");
	//char* buf;
	if (IniFile != NULL)
	{                              
		fprintf(IniFile,"%s %f\n","initial_pose_x=",pose_odom.x);
		fprintf(IniFile,"%s %f\n","initial_pose_y=",pose_odom.y);
		fprintf(IniFile,"%s %f\n","initial_pose_a=",pose_odom.theta);	
		//printf("写入文件中的初始位姿为   %f   %f   %f\n",pose_odom.x,pose_odom.y,pose_odom.theta);	
	}
	else printf("文件打开失败");
	fclose(IniFile);*/
    return true;
}
//--------------------------------------------------------------------

AGVInstance *instance = nullptr;

void init_global_agv_instance()
{
    if (instance == nullptr)
    {
        instance = new AGVInstance();
    }
}

AGVInstance *get_global_agv_instance()
{
    return instance;
}

storage *get_global_storage()
{
    return instance->st;
}
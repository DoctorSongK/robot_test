#include "ridar_to_obavoid.h"
#include "../logic/agv.h"
#include <math.h>
#include <stdio.h>
#include <../common/warn_err_data.h>

//FILE *fp;
AGV_Obavoid::AGV_Obavoid(MsgQueue *msg_queue, int rec_channel_, int send_channel_)
{
        enable_obavoid = (get_configs()->get_int("obavoid_3d", "enable_obavoid", nullptr) == 0) ? false : true;

        if (get_configs()->get_int("obavoid_3d", "obavoid_strategy", nullptr) == 0)
        {
                obavoid_strategy = normal;
        }
        else if (get_configs()->get_int("obavoid_3d", "obavoid_strategy", nullptr) == 1)
        {
                obavoid_strategy = front_back_self_adaption;
        }
        else if (get_configs()->get_int("obavoid_3d", "obavoid_strategy", nullptr) == 2)
        {
                obavoid_strategy = all_self_adaption;
        }

        if (get_configs()->get_int("obavoid_3d", "obavoid_region", nullptr) == 0)
        {
                obavoid_region = front_rec;
        }
        else if (get_configs()->get_int("obavoid_3d", "obavoid_region", nullptr) == 1)
        {
                obavoid_region = front_sector;
        }
        else if (get_configs()->get_int("obavoid_3d", "obavoid_region", nullptr) == 2)
        {
                obavoid_region = front_back_rec;
        }
        else if (get_configs()->get_int("obavoid_3d", "obavoid_region", nullptr) == 3)
        {
                obavoid_region = front_back_sector;
        }
        else if (get_configs()->get_int("obavoid_3d", "obavoid_region", nullptr) == 4)
        {
                obavoid_region = rec_and_rec;
        }
        else if (get_configs()->get_int("obavoid_3d", "obavoid_region", nullptr) == 5)
        {
                obavoid_region = rec_and_sector;
        }
        // enable_obavoid = true;
        // obavoid_strategy = normal;
        // obavoid_region = front_rec;
        // rotate_time = 0;

        installation_position_x = get_configs()->get_float("radar", "installation_position_x", nullptr); //0.20
        installation_position_y = get_configs()->get_float("radar", "installation_position_y", nullptr);
        installation_position_z = get_configs()->get_float("radar", "installation_position_z", nullptr); //0.339??????????????????????????????????????????????????????????????????-installation_position_z

        range_min_ = get_configs()->get_float("obavoid_3d", "range_min_", nullptr);
        range_max_ = get_configs()->get_float("obavoid_3d", "range_max_", nullptr);
        max_height_pc = get_configs()->get_float("obavoid_3d", "max_height_pc", nullptr);
        robot_length = 0.58;
        robot_width = 0.58;
        judge_threshold = get_configs()->get_float("obavoid_3d", "judge_threshold", nullptr);
        collision_threshold = get_configs()->get_int("obavoid_3d", "collision_threshold", nullptr);

        // range_min_ = 0.1;
        // range_max_ = 4;

        // max_height_pc = 1.0;
        // robot_length = 0.58;
        // robot_width = 0.58;
        // judge_threshold = 0.6;
        // collision_threshold = 100;

        //gridmap = new SimpleGridMap();
        region_width_ = get_configs()->get_float("obavoid_3d", "region_width", nullptr);
        region_length_ = get_configs()->get_float("obavoid_3d", "region_length", nullptr);
        // region_width_ = 3 * robot_width;
        // region_length_ = 3 * robot_width;
        count_ = 10;

        enable_speed_compensation = (get_configs()->get_int("obavoid_3d", "enable_speed_compensation", nullptr) == 0) ? false : true;
        // enable_speed_compensation = false;

        init_obavoid();
        this->ptr_msg_queue = msg_queue;
        rec_channel = rec_channel_;
        obavoid_c = send_channel_;
        ptr_msg_queue->add_listener(rec_channel, this);
        pre_timestamp = -100000000000;
        agv_run_environment.resize(4);

        use_intensity = (get_configs()->get_int("obavoid_3d", "use_intensity", nullptr) == 0) ? false : true;                   //???????????????????????????
        use_voxel_filter = (get_configs()->get_int("obavoid_3d", "use_voxel_filter", nullptr) == 0) ? false : true;             //????????????????????????
        use_statistical_filter = (get_configs()->get_int("obavoid_3d", "use_statistical_filter", nullptr) == 0) ? false : true; //????????????????????????
        use_remove_ground = (get_configs()->get_int("obavoid_3d", "use_remove_ground", nullptr) == 0) ? false : true;           //???????????????????????????

        // use_intensity = false;//???????????????????????????
        // use_voxel_filter = false;//????????????????????????
        // use_statistical_filter = false;//????????????????????????
        // use_remove_ground = true;//???????????????????????????

        /*????????????*/
        voxel_unit_size = 0.15;         //??????????????????0.05
        k_nearest_neighbor_size = 50.0; //????????????k???????????????
        standard_deviation = 1.0;       //?????????????????????

        radar_driver_angle = get_configs()->get_float("obavoid_3d", "radar_driver_angle", nullptr);
        concentric_divider_distance_ = get_configs()->get_float("obavoid_3d", "concentric_divider_distance_", nullptr);
        local_max_slope_ = get_configs()->get_float("obavoid_3d", "local_max_slope_", nullptr);
        general_max_slope_ = get_configs()->get_float("obavoid_3d", "general_max_slope_", nullptr);
        min_height_threshold_ = get_configs()->get_float("obavoid_3d", "min_height_threshold_", nullptr);
        reclass_distance_threshold_ = get_configs()->get_float("obavoid_3d", "reclass_distance_threshold_", nullptr);
        radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

        // voxel_unit_size = 0.15;         //??????????????????0.05
        // k_nearest_neighbor_size = 50.0; //????????????k???????????????
        // standard_deviation = 1.0;       //?????????????????????

        // radar_driver_angle = 0.18;
        // concentric_divider_distance_ = 0.01;
        // local_max_slope_ = 6;
        // general_max_slope_ = 4;
        // min_height_threshold_ = 0.05;
        // reclass_distance_threshold_ = 0.2;
        // radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

        voxelgrid_output.reset(new pcl::PointCloud<pcl::PointXYZI>());
        statistical_output.reset(new pcl::PointCloud<pcl::PointXYZI>());
        //   pthread_create(&obavoid_thread, NULL, thread_function, this);
        //fp = fopen("ob_radar_data.txt","wb");
        pthread_mutex_init(&agv_ob_3d, nullptr);
}

AGV_Obavoid::~AGV_Obavoid()
{
        //   pthread_join(obavoid_thread, NULL);
        pthread_mutex_destroy(&agv_ob_3d);
}

double AGV_Obavoid::get_distance(double x, double y)
{
        return sqrt(x * x + y * y);
}

void AGV_Obavoid::pcl_point_cloud2custom_point_cloud(pcl::PointCloud<pcl::PointXYZI> msg_data) ///////////////////???????????????????????????????????????
{
        /*????????????*/
        //memset????????????extern void *memset(void *buffer,int c,int count);??????buffer????????????????????????c?????????buffer?????????count???buffer?????????
        //memcpy???????????????????????????????????????????????????????????????????????????????????????????????????????????????strcpy????????????????????????????????????/0?????????????????????
        //?????????char a[100],b[50];memcpy(b,a,sizeof(b))????????????a???sizeof(b)??????????????????b?????????????????????sizeof(a),?????????b?????????????????????
        //char a[100],b[50];strcpy(a,b);??????strcpy(b,a),?????????a?????????????????????????????????'/0'?????????????????????50??????????????????????????????b?????????????????????
        memset(&custom_data_3d, 0, sizeof(custom_point_cloud));
        custom_data_3d.height = msg_data.height;
        custom_data_3d.width = msg_data.width;
        custom_data_3d.timestamp = msg_data.header.stamp;
        auto iter = msg_data.points.begin();
	std::cout<<"6666666666666666666666666666666666666   "<<msg_data.points.size()<<std::endl;
        for (; iter != msg_data.points.end(); iter++)
        {
                //fprintf(fp, "%f ,%f ,%f \n",iter->x,iter->y,iter->z);
                if (iter->z > max_height_pc || iter->z < (-installation_position_z - 0.01)) //???????????????
                        continue;
                if (std::isnan(iter->x) || std::isnan(iter->y) || std::isnan(iter->z)) //??????nan???
                        continue;
                if ((iter->x < robot_length / 2 - abs(installation_position_x)) && (iter->x > -(robot_length / 2 + installation_position_x)) && (iter->y > -robot_width / 2) && (iter->y < robot_width / 2)) //????????????????????????????????????
                        continue;
                double range = get_distance(iter->x, iter->y);
                if (range < range_min_ || range > range_max_)
                        continue;
                PointXYZI point;
                point.x = iter->x;
                point.y = iter->y;
                point.z = iter->z;
                point.intensity = iter->intensity;
                custom_data_3d.point_cloud.push_back(point);
        }

        // for(int i = 0;i < msg_data.points.size();i++){
        //         if(msg_data.points[i].z > max_height_pc||msg_data.points[i].z < (-installation_position_z-0.01))
        //                 continue;
        //         if(std::isnan(msg_data.points[i].x)||std::isnan(msg_data.points[i].y)||std::isnan(msg_data.points[i].z))//??????nan???
        //                 continue;
        //         if((msg_data.points[i].x < robot_length/2-abs(installation_position_x)) && (msg_data.points[i].x > -(robot_length/2+installation_position_x)) && (msg_data.points[i].y > -robot_width/2) && (msg_data.points[i].y < robot_width/2 ))
        //                 continue;
        //         double range = get_distance(msg_data.points[i].x, msg_data.points[i].y);
        //         if (range < range_min_||range > range_max_)
        //                 continue;
        //         PointXYZI point;
        //         point.x = msg_data.points[i].x;
        //         point.y = msg_data.points[i].y;
        //         point.z = msg_data.points[i].z;
        //         point.intensity = msg_data.points[i].intensity;
        //         custom_data_3d.point_cloud.push_back(point);
        // }
        //std::cout<<"custom_data_3d.point_cloud.size()   "<<custom_data_3d.point_cloud.size()<<std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr AGV_Obavoid::voxel_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr msg)
{
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxelgrid_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(msg);
        sor.setLeafSize(voxel_unit_size, voxel_unit_size, voxel_unit_size); //???????????????????????????????????????5??????
        // apply filter
        sor.filter(*voxelgrid_filtered);
        return voxelgrid_filtered;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr AGV_Obavoid::statistical_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr msg)
{
        pcl::PointCloud<pcl::PointXYZI>::Ptr statistical_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(msg);
        sor.setMeanK(k_nearest_neighbor_size);
        sor.setStddevMulThresh(standard_deviation);
        sor.filter(*statistical_filtered);
        return statistical_filtered;
}

void AGV_Obavoid::XYZI_to_RTZColor()
{
        //    cout<<"????????????"<<msg.point_cloud.size()<<endl;
        //    out_organized_points.resize(msg.point_cloud.size());
        out_radial_divided_indices.clear();
        out_radial_ordered_clouds.clear();
        out_radial_divided_indices.resize(radial_dividers_num_);
        out_radial_ordered_clouds.resize(radial_dividers_num_);

        for (int i = 0; i < custom_data_3d.point_cloud.size(); i++)
        {
                PointXYZIRTColor new_point;
                auto radius = (float)sqrt(custom_data_3d.point_cloud[i].x * custom_data_3d.point_cloud[i].x + custom_data_3d.point_cloud[i].y * custom_data_3d.point_cloud[i].y); //??????
                auto theta = (float)atan2(custom_data_3d.point_cloud[i].y, custom_data_3d.point_cloud[i].x) * 180 / M_PI;                                                         //??????
                if (theta < 0)
                {
                        theta += 360;
                } //???????????????0-360
                //floor(x)??????????????????????????????????????????????????????
                auto radial_div = (size_t)floor(theta / radar_driver_angle); //?????????????????????0.18???
                //fabs(x)???????????????????????????????????????????????????(??????????????????????????????)
                auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_)); //??????????????????0.01m

                new_point.point = custom_data_3d.point_cloud[i]; //pcl_point
                new_point.radius = radius;
                new_point.theta = theta;
                new_point.radial_div = radial_div;
                new_point.concentric_div = concentric_div;
                new_point.original_index = i;

                //out_organized_points[i] = new_point;

                //??????????????????????????????????????????
                out_radial_divided_indices[radial_div].indices.push_back(i);
                out_radial_ordered_clouds[radial_div].push_back(new_point); //??????
        }

        //?????????????????????????????????????????????????????????,?????????out_radial_ordered_clouds[radial_div]???
        //#pragma omp parallel for//????????????
        for (int j = 0; j < radial_dividers_num_; j++)
        {
                std::sort(out_radial_ordered_clouds[j].begin(), out_radial_ordered_clouds[j].end(), [](const PointXYZIRTColor &a, const PointXYZIRTColor &b)
                          { return a.radius < b.radius; });
        }

        //sort???????????????????????????????????????
}
void AGV_Obavoid::remove_ground_points()
{
        std::vector<PointXYZI> new_point_cloud = {};
        std::vector<PointXYZI> out_ground_point_cloud = {};
        //   custom_point_cloud processed_data;
        if (out_radial_ordered_clouds.size() == 0)
                std::cout << "??????????????????\n";
        //memset(&custom_data_3d,0,sizeof(custom_point_cloud));
        custom_data_3d.point_cloud.clear();
        out_ground_indices.indices.clear();
        out_no_ground_indices.indices.clear();
        //#pragma omp parallel for
        //?????????????????????
        for (size_t i = 0; i < out_radial_ordered_clouds.size(); i++)
        {
                float prev_radius = 0.f;
                float prev_height = -installation_position_z;
                bool prev_ground = false;
                bool current_ground = false;
                //  ????????????????????????????????????
                for (size_t j = 0; j < out_radial_ordered_clouds[i].size(); j++)
                {
                        float points_distance = out_radial_ordered_clouds[i][j].radius - prev_radius;
                        // DEG2RAD()???????????????????????????????????????local_max_slope_???????????????????????????????????????
                        float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance; //??????????????????????????????????????????????????????????????????????????????8???
                        float current_height = out_radial_ordered_clouds[i][j].point.z;
                        // general_max_slope_???????????????????????????????????????????????????????????????5???
                        float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * out_radial_ordered_clouds[i][j].radius;

                        //????????????????????????????????????????????????????????????????????????
                        if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
                        {
                                height_threshold = min_height_threshold_;
                        }

                        //??????local?????????????????????????????????????????????
                        if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
                        {
                                //?????????????????????????????????????????????????????????????????????????????????????????????
                                if (!prev_ground)
                                {
                                        if (current_height <= (-installation_position_z + general_height_threshold) && current_height >= (-installation_position_z - general_height_threshold))
                                                current_ground = true;
                                        else
                                                current_ground = false;
                                }
                                else
                                        current_ground = true;
                        }
                        else
                        {
                                //??????????????????????????????????????????????????????????????????
                                if (points_distance > reclass_distance_threshold_ && (current_height <= (-installation_position_z + height_threshold) && current_height >= (-installation_position_z - height_threshold)))
                                        current_ground = true;
                                else
                                        current_ground = false;
                        }
                        if (current_ground)
                        {
                                new_point_cloud.push_back(out_radial_ordered_clouds[i][j].point);
                                out_ground_indices.indices.push_back(out_radial_ordered_clouds[i][j].original_index);
                                prev_ground = true;
                        }
                        else
                        {
                                out_ground_point_cloud.push_back(out_radial_ordered_clouds[i][j].point);
                                out_no_ground_indices.indices.push_back(out_radial_ordered_clouds[i][j].original_index);
                                prev_ground = false;
                        }
                        prev_radius = out_radial_ordered_clouds[i][j].radius;
                        prev_height = out_radial_ordered_clouds[i][j].point.z;
                }
        }
        custom_data_3d.point_cloud.assign(out_ground_point_cloud.begin(), out_ground_point_cloud.end());
        //std::cout<<"custom_data_3d.point_cloud.size()"<<" "<<custom_data_3d.point_cloud.size()<<std::endl;
}

void AGV_Obavoid::init_obavoid()
{
        switch (obavoid_region)
        {
        case front_rec:
                avoid_dis1y = 0.2;
                avoid_dis1x = 0.4;//??????0.4
                avoid_dis2y = 0.2;
                avoid_dis2x = 0.6;
                avoid_dis3y = 0.2;
                avoid_dis3x = 1.0;
                avoid_lside = 0.2;
                avoid_rside = 0.2;
                break;
        case front_sector:
                break;
        case front_back_rec:
                avoid_dis1y = 0.2;
                avoid_dis1x = 0.4;
                avoid_dis2y = 0.2;
                avoid_dis2x = 0.6;
                avoid_dis3y = 0.2;
                avoid_dis3x = 1.0;
                break;
        case front_back_sector:
                break;
        case rec_and_rec:
                avoid_dis1y = 0.2;
                avoid_dis1x = 0.4;
                avoid_dis2y = 0.2;
                avoid_dis2x = 0.6;
                avoid_dis3y = 0.2;
                avoid_dis3x = 1.0;
                avoid_lside = 0.2;
                avoid_rside = 0.2;
                break;
        case rec_and_sector:
                break;
        default:
                std::cout << "Input obavoid_region error..." << std::endl;
                break;
        }
        agv_obavoid_level = normal_run;
        if (!enable_speed_compensation)
                speed_compensation = 0.0;
}

void AGV_Obavoid::chose_agv_obstrategy(int time)
{
        switch (obavoid_strategy)
        {
        case normal:
        {
                std::cout << "????????????normal..." << std::endl;
                portrait_adjust_level = 1;
                transverse_adjust_level = 1;
        }
        break;
        case front_back_self_adaption:
        {
                //??????????????????????????????
                if (time >= 2)
                {
                        std::cout<<"????????????????????????1111111..."<<std::endl;
                        portrait_adjust_level = 2;
                }
                else
                        portrait_adjust_level = 1;
                transverse_adjust_level = 1;
        }
        break;
        case all_self_adaption:
        {
                if (time >= 2)
                {
                        std::cout<<"????????????????????????2222222..."<<std::endl;
                        portrait_adjust_level = 2;
                        transverse_adjust_level = 2;
                }
                else
                {
                        portrait_adjust_level = 1;
                        transverse_adjust_level = 1;
                }
        }
        break;
        }
}

void AGV_Obavoid::obavoid_state_output(obavoid_state &agv_obavoid_state)
{
        agv_obavoid_state = {4, 4, 4, 4};
        switch (obavoid_region)
        {
        case front_rec:
        {
                if (colli_cntf1 > collision_threshold)
                {
                        agv_obavoid_level = emergency_braking;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else if (colli_cntf2 > collision_threshold)
                {
                        agv_obavoid_level = slow_down;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else if (colli_cntf3 > collision_threshold)
                {
                        agv_obavoid_level = warning;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else
                {
                        agv_obavoid_level = normal_run;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
        }
        break;
        case front_back_rec:
        {
                if (colli_cntf1 > collision_threshold)
                {
                        agv_obavoid_level = emergency_braking;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else if (colli_cntf2 > collision_threshold)
                {
                        agv_obavoid_level = slow_down;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else if (colli_cntf3 > collision_threshold)
                {
                        agv_obavoid_level = warning;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else
                {
                        agv_obavoid_level = normal_run;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }

                if (colli_cntb1 > collision_threshold)
                {
                        agv_obavoid_level = emergency_braking;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }
                else if (colli_cntb2 > collision_threshold)
                {
                        agv_obavoid_level = slow_down;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }
                else if (colli_cntb3 > collision_threshold)
                {
                        agv_obavoid_level = warning;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }
                else
                {
                        agv_obavoid_level = normal_run;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }
        }
        break;
        case rec_and_rec:
        {
                if (colli_cntf1 > collision_threshold)
                {
                        agv_obavoid_level = emergency_braking;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else if (colli_cntf2 > collision_threshold)
                {
                        agv_obavoid_level = slow_down;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else if (colli_cntf3 > collision_threshold)
                {
                        agv_obavoid_level = warning;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }
                else
                {
                        agv_obavoid_level = normal_run;
                        agv_obavoid_state.ahead_obstacle = agv_obavoid_level;
                }

                if (colli_cntb1 > collision_threshold)
                {
                        agv_obavoid_level = emergency_braking;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }
                else if (colli_cntb2 > collision_threshold)
                {
                        agv_obavoid_level = slow_down;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }
                else if (colli_cntb3 > collision_threshold)
                {
                        agv_obavoid_level = warning;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }
                else
                {
                        agv_obavoid_level = normal_run;
                        agv_obavoid_state.rear_obstacle = agv_obavoid_level;
                }

                if (colli_cntr > collision_threshold)
                {
                        agv_obavoid_level = emergency_braking;
                        agv_obavoid_state.right_obstacle = agv_obavoid_level;
                }
                else
                {
                        agv_obavoid_level = normal_run;
                        agv_obavoid_state.right_obstacle = agv_obavoid_level;
                }

                if (colli_cntl > collision_threshold)
                {
                        agv_obavoid_level = emergency_braking;
                        agv_obavoid_state.left_obstacle = agv_obavoid_level;
                }
                else
                {
                        agv_obavoid_level = normal_run;
                        agv_obavoid_state.left_obstacle = agv_obavoid_level;
                }
        }
        break;
        case front_sector:
        case front_back_sector:
        case rec_and_sector:
                std::cout << "???????????????..." << std::endl;
        }

        //  cout << agv_obavoid_state.ahead_obstacle << " " << agv_obavoid_state.rear_obstacle << " " << agv_obavoid_state.left_obstacle << " " << agv_obavoid_state.right_obstacle << endl;
}
void AGV_Obavoid::sense_robot_environment(Position agv_pose, float region_width, float region_length, int count)
{
        if (gridmap == nullptr)
                return;
        std::vector<int> robot_map_pose(2);
        int map_size = gridmap->map_info.width * gridmap->map_info.height;
        printf("map_origen_x:%lf,map_origen_y:%lf\n",gridmap->map_info.origen_x,gridmap->map_info.origen_y);
        robot_map_pose[0] = gridmap->map_info.global_x_to_map_x(agv_pose.x);
        robot_map_pose[1] = gridmap->map_info.global_y_to_map_y(agv_pose.y);
        //       printf("??????????????????????????????????????????:robot_map_pose(%d,%d)\n",robot_map_pose[0],robot_map_pose[1]);
        //       printf("???????????????????????????????????????????????????:robot_map_pose(%d,%d)\n",gridmap->map_info.global_x_to_map_x(agv_pose.x+0.04),gridmap->map_info.global_y_to_map_y(agv_pose.y));
        //       printf("???????????????????????????????????????????????????:robot_map_pose(%d,%d)\n",gridmap->map_info.global_x_to_map_x(agv_pose.x-0.04),gridmap->map_info.global_y_to_map_y(agv_pose.y));
        //       printf("???????????????????????????????????????????????????:robot_map_pose(%d,%d)\n",gridmap->map_info.global_x_to_map_x(agv_pose.x),gridmap->map_info.global_y_to_map_y(agv_pose.y+0.04));
        //       printf("???????????????????????????????????????????????????:robot_map_pose(%d,%d)\n",gridmap->map_info.global_x_to_map_x(agv_pose.x),gridmap->map_info.global_y_to_map_y(agv_pose.y-0.04));

        double resolution = gridmap->map_info.resolution; //?????? m
        int wstep = std::floor(region_width / (count * resolution));
        int lstep = std::floor(region_length / (count * resolution));

        int width_limit = region_length / resolution;
        int length_limit = region_length / resolution;
       // printf("???????????????region_width:%d,region_length:%d,width_limit:%d,length_limit:%d\n",wstep,lstep,width_limit,length_limit);
        //printf("???????????????region_width:%lf,region_length:%lf,width_limit:%d,length_limit:%d\n",wstep,lstep,width_limit,length_limit);
        ///??????????????????????????????,agv_run_environment???????????????????????????????????????
        for (int i = 0; i < (width_limit / 2); i += wstep)
        {
                std::vector<int> find_pose_left = {robot_map_pose[0]+i, robot_map_pose[1]};
                std::vector<int> find_pose_right = {robot_map_pose[0]-i, robot_map_pose[1]};
                //     printf("find_pose_left(%d,%d)\n",find_pose_left[0],find_pose_left[1]);
                //     printf("find_pose_right(%d,%d)\n",find_pose_right[0],find_pose_right[1]);
                bool front1_detected = false;
                bool front2_detected = false;
                bool back1_detected = false;
                bool back2_detected = false;
                int front_data1 = 0;
                int front_data2 = 0;
                int back_data1 = 0;
                int back_data2 = 0;
                int curent_left_grayscale = gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_left[0], find_pose_left[1])];
                int curent_right_grayscale = gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_right[0], find_pose_right[1])];
                for (int j = 1; j < length_limit / 2; j++)
                {
                        // printf("???????????????????????????????????????%d\n",(int)(gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_left[0]+j, find_pose_left[1])] & 0xff));
                        if (!(curent_left_grayscale > 60 && curent_left_grayscale<255)&&gridmap->map_info.map_xy_to_array_index(find_pose_left[0], find_pose_left[1]+j) < map_size && !front1_detected)
                        {
                                front_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_left[0], find_pose_left[1]+j)] & 0xff);
                                //int front_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_left[0] + j, find_pose_left[1])] & 0xff);
                                if (front_data1 > 60 && front_data1 < 255)
                                {
                                        agv_run_environment[0]++;
                                        front1_detected = true;
                                }
                        }
                        if (!(curent_left_grayscale > 60&&curent_left_grayscale<255)&&gridmap->map_info.map_xy_to_array_index(find_pose_left[0], find_pose_left[1]-j) < map_size && !front2_detected)
                        {
                                back_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_left[0], find_pose_left[1]-j)] & 0xff);
                                //int back_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_left[0] - j, find_pose_left[1])] & 0xff);
                                if (back_data1 > 60 && back_data1 < 255)
                                {
                                        agv_run_environment[1]++;
                                        front2_detected = true;
                                }
                        }
                        if (!(curent_right_grayscale > 60&&curent_right_grayscale<255)&&gridmap->map_info.map_xy_to_array_index(find_pose_right[0], find_pose_right[1]+j) < map_size && !back1_detected)
                        {
                                front_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_right[0], find_pose_right[1]+j)] & 0xff);
                                //int front_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_right[0] + j, find_pose_right[1])] & 0xff);
                                if (front_data2 > 60 && front_data2 < 255)
                                {
                                        agv_run_environment[0]++;
                                        back1_detected = true;
                                }
                        }
                        if (!(curent_right_grayscale > 60&&curent_right_grayscale<255)&&gridmap->map_info.map_xy_to_array_index(find_pose_right[0], find_pose_right[1]-j) < map_size && !back2_detected)
                        {
                                back_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_right[0], find_pose_right[1]-j)] & 0xff);
                                //int back_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_right[0] - j, find_pose_right[1])] & 0xff);
                                if (back_data2 > 60 && back_data2 < 255)
                                {
                                        agv_run_environment[1]++;
                                        back2_detected = true;
                                }
                        }
                       // printf("?????????????????? %d,%d,%d,%d\n", front_data1, back_data1, front_data2, back_data2);
                        front_data1 = 0;
                        back_data1 = 0;
                        front_data2 = 0;
                        back_data2 = 0;
                        front1_detected = false;
                        front2_detected = false;
                        back1_detected = false;
                        back2_detected = false;
                }
        }

        ///??????????????????????????????,agv_run_environment???????????????????????????????????????
        for (int i = 0; i < (length_limit / 2); i += lstep)
        {
                std::vector<int> find_pose_front = {robot_map_pose[0], robot_map_pose[1] + i};
                std::vector<int> find_pose_back = {robot_map_pose[0], robot_map_pose[1] - i};
                //     printf("find_pose_front(%d,%d)\n",find_pose_front[0],find_pose_front[1]);
                //     printf("find_pose_back(%d,%d)\n",find_pose_back[0],find_pose_back[1]);
                bool left1_detected = false;
                bool left2_detected = false;
                bool right1_detected = false;
                bool right2_detected = false;

                int left_data1 = 0;
                int right_data1 = 0;
                int left_data2 = 0;
                int right_data2 = 0;

                int curent_front_grayscale = gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_front[0], find_pose_front[1])];
                int curent_back_grayscale = gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_back[0], find_pose_back[1])];

                for (int j = 1; j < width_limit / 2; j++)
                {
                        if (!(curent_front_grayscale > 60&&curent_front_grayscale<255) && gridmap->map_info.map_xy_to_array_index(find_pose_front[0] + j, find_pose_front[1]) < map_size && !left1_detected)
                        {
                                left_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_front[0] + j, find_pose_front[1])] & 0xff);
                                // int left_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_front[0], find_pose_front[1] + j)] & 0xff);
                                if (left_data1 > 60 && left_data1 < 255){
                                        left1_detected = true;
                                        agv_run_environment[2]++;
                                }       
                        }
                        if (!(curent_front_grayscale > 60&&curent_front_grayscale<255) && gridmap->map_info.map_xy_to_array_index(find_pose_front[0] - j, find_pose_front[1]) < map_size && !right1_detected)
                        {
                                right_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_front[0] - j, find_pose_front[1])] & 0xff);
                                //int right_data1 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_front[0], find_pose_front[1] - j)] & 0xff);
                                if (right_data1 > 60 && right_data1 < 255){
                                        agv_run_environment[3]++;
                                        right1_detected = true;
                                }
                                        
                        }
                        if (!(curent_back_grayscale > 60&&curent_back_grayscale<255) && gridmap->map_info.map_xy_to_array_index(find_pose_back[0] + j , find_pose_back[1]) < map_size && !left2_detected)
                        {
                                left_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_back[0] + j, find_pose_back[1])] & 0xff);
                                //int left_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_back[0], find_pose_back[1] + j)] & 0xff);
                                if (left_data2 > 60 && left_data2 < 255){
                                        agv_run_environment[2]++;
                                        left2_detected = true;
                                }
                                       
                        }
                        if (!(curent_back_grayscale > 60&&curent_back_grayscale<255) && gridmap->map_info.map_xy_to_array_index(find_pose_back[0] - j, find_pose_back[1]) < map_size && !right2_detected)
                        {
                                right_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_back[0] - j, find_pose_back[1])] & 0xff);
                                //int right_data2 = (gridmap->datas[gridmap->map_info.map_xy_to_array_index(find_pose_back[0], find_pose_back[1] - j)] & 0xff);
                                if (right_data2 > 60 && right_data2 < 255){
                                        agv_run_environment[3]++;
                                        right_data2 = true;
                                }
                                        
                        }
                       // printf("?????????????????? %d,%d,%d,%d\n", left_data1, right_data1, left_data2, right_data2);
                        left_data1 = 0; right_data1 = 0; left_data2 = 0; right_data2 = 0;
                        left1_detected = false;
                        left2_detected = false;
                        right1_detected = false;
                        right2_detected = false;
                }
        }
        // printf("%d %d %d %d\n",agv_run_environment[0],agv_run_environment[1],agv_run_environment[2],agv_run_environment[3]);
        for (int i = 0; i < 4; i++)
        {
                if ((agv_run_environment[i] / count) > judge_threshold)
                        agv_run_environment[i] = 1;
                else
                        agv_run_environment[i] = 0;
        }
}

void AGV_Obavoid::get_agv_obavoid_state()
{
        colli_cntf1 = 0;
        colli_cntf2 = 0;
        colli_cntf3 = 0;
        colli_cntl = 0;
        colli_cntr = 0;
        colli_cntb1 = 0;
        colli_cntb2 = 0;
        colli_cntb3 = 0;
        //std::cout<<custom_data_3d.point_cloud.size()<<std::endl;
        auto iter = custom_data_3d.point_cloud.begin();
        for (; iter != custom_data_3d.point_cloud.end(); iter++)
        {
                // fprintf(fp, "%f ,%f ,%f \n",iter->x,iter->y,iter->z);
                // switch (obavoid_region)
                // {
                // case front_rec:
                // case front_back_rec:
                // case rec_and_rec:
                // {
                // printf("???????????????%f\n",(robot_length / 2 - installation_position_x + avoid_dis1x / portrait_adjust_level + speed_compensation));
                // std::cout<<"x: "<<iter->x<<"y: "<<iter->y<<std::endl;
                //??????????????????????????????????????????????????????????????????????????????y??????????????????????????????????????????installation_position_x???installation_position_y??????????????????
		float nearest_point = avoid_dis1x / portrait_adjust_level;
		if(nearest_point < 0.4) nearest_point = 0.4;
                if (iter->x > (robot_length / 2 - installation_position_x + 0.) && iter->x < (robot_length / 2 - installation_position_x + nearest_point + speed_compensation) && iter->y > (0 - (robot_width / 2 + installation_position_y + avoid_dis1y / portrait_adjust_level + speed_compensation)) && iter->y < (robot_width / 2 - installation_position_y + avoid_dis1y / portrait_adjust_level + speed_compensation))
                {
                        colli_cntf1++;
                        //printf("???????????????1??????................\n");
                }
                else if (iter->x > (robot_length / 2 - installation_position_x + 0.) && iter->x < (robot_length / 2 - installation_position_x + avoid_dis2x / portrait_adjust_level + speed_compensation) && iter->y > (0 - (robot_width / 2 + installation_position_y + avoid_dis2y / portrait_adjust_level + speed_compensation)) && iter->y < (robot_width / 2 - installation_position_y + avoid_dis2y / portrait_adjust_level + speed_compensation))
                {
                        colli_cntf2++;
                        //printf("???????????????2??????................\n");
                }
                else if (iter->x > (robot_length / 2 - installation_position_x + 0.) && iter->x < (robot_length / 2 - installation_position_x + avoid_dis3x / portrait_adjust_level + speed_compensation) && iter->y > (0 - (robot_width / 2 + installation_position_y + avoid_dis3y / portrait_adjust_level + speed_compensation)) && iter->y < (robot_width / 2 - installation_position_y + avoid_dis3y / portrait_adjust_level + speed_compensation))
                {
                        colli_cntf3++;
                        // printf("???????????????3??????................\n");
                }
                else if (iter->x < (0 - (robot_length / 2 + installation_position_x + 0.)) && iter->x > (0 - (robot_length / 2 + installation_position_x + nearest_point + speed_compensation)) && iter->y > (0 - (robot_width / 2 + installation_position_y + avoid_dis1y / portrait_adjust_level + speed_compensation)) && iter->y < (robot_width / 2 - installation_position_y + avoid_dis1y / portrait_adjust_level + speed_compensation))
                {
                        colli_cntb1++;
                        // printf("???????????????1??????................\n");
                }
                else if (iter->x < (0 - (robot_length / 2 + installation_position_x + 0.)) && iter->x > (0 - (robot_length / 2 + installation_position_x + avoid_dis2x / portrait_adjust_level + speed_compensation)) && iter->y > (0 - (robot_width / 2 + installation_position_y + avoid_dis2y / portrait_adjust_level + speed_compensation)) && iter->y < (robot_width / 2 - installation_position_y + avoid_dis2y / portrait_adjust_level + speed_compensation))
                {
                        colli_cntb2++;
                        // printf("???????????????2??????................\n");
                }
                else if (iter->x < (0 - (robot_length / 2 + installation_position_x + 0.)) && iter->x > (0 - (robot_length / 2 + installation_position_x + avoid_dis3x / portrait_adjust_level + speed_compensation)) && iter->y > (0 - (robot_width / 2 + installation_position_y + avoid_dis3y / portrait_adjust_level + speed_compensation)) && iter->y < (robot_width / 2 - installation_position_y + avoid_dis3y / portrait_adjust_level + speed_compensation))
                {
                        colli_cntb3++;
                        //printf("???????????????3??????................\n");
                }
                else if (iter->x < (robot_length / 2 - installation_position_x) && iter->x > (0 - robot_length / 2 - installation_position_x) && iter->y > (robot_width / 2 - installation_position_y + 0.) && iter->y < (robot_width / 2 - installation_position_y + avoid_lside / transverse_adjust_level))
                {
                        colli_cntl++;
                        // printf("?????????????????????................\n");
                }
                else if (iter->x < (robot_length / 2 - installation_position_x) && iter->x > (0 - robot_length / 2 - installation_position_x) && iter->y < (0 - (robot_width / 2 + installation_position_y + 0.)) && iter->y > (0 - (robot_width / 2 + installation_position_y + avoid_rside / transverse_adjust_level)))
                {
                        colli_cntr++;
                        //  printf("?????????????????????................\n");
                }
                // }
                // break;
                // case front_sector:
                // case front_back_sector:
                // case rec_and_sector:
                // {
                // }
                // break;
                // default:
                //         std::cout << "?????????????????????????????????..." << std::endl;
                // }
        }
}

void AGV_Obavoid::recieve_message(const int channel, char *buf, const int size)
{
        if (channel == CHANNEL_3D_TO_OB_RAW)
        {
                // if(rotate_time++>2){
                pthread_mutex_lock(&agv_ob_3d);
                PCL_PointCloudData data;
                data.from_char_array(buf, size);
                handle_agv_obavoid(data);
                pthread_mutex_unlock(&agv_ob_3d);
                // rotate_time = 0;
                //  }
        }
}

void AGV_Obavoid::handle_agv_obavoid(PCL_PointCloudData point_data)
{
        if (point_data.pcl_pointcloud.points.size() < 100)
                return;
        pcl::PointCloud<pcl::PointXYZI> new_msg;
        if ((point_data.pcl_pointcloud.header.stamp) == pre_timestamp)
        {
                return;
        }
        else
        {
                if (recieved_raw_data.empty())
                {
                        pre_timestamp = point_data.pcl_pointcloud.header.stamp;
                        new_msg = point_data.pcl_pointcloud;
                }
                else
                {
                        pre_timestamp = point_data.pcl_pointcloud.header.stamp;
                        recieved_raw_data.push_back(point_data.pcl_pointcloud);
                        new_msg = recieved_raw_data.back();
                        if (recieved_raw_data.size() > 2)
                                recieved_raw_data.clear();
                }
        }
        if (use_voxel_filter)
        {
                voxelgrid_output = voxel_filter(new_msg.makeShared());
        }
        if (use_statistical_filter)
        {
                if (use_voxel_filter)
                        statistical_output = statistical_filter(voxelgrid_output);
                else
                {
                        statistical_output = statistical_filter(new_msg.makeShared());
                }
        }

        if ((use_voxel_filter && use_statistical_filter) || (use_statistical_filter && !use_voxel_filter))
                pcl_point_cloud2custom_point_cloud(*statistical_output);
        else if (use_voxel_filter)
                pcl_point_cloud2custom_point_cloud(*voxelgrid_output);
        else
                pcl_point_cloud2custom_point_cloud(new_msg);
        if (use_remove_ground)
        {
                XYZI_to_RTZColor();
                remove_ground_points();
        }

        obavoid_state ob_state;
        int agv_state = get_global_agv_instance()->get_system_state();
        if (enable_obavoid)
        {
                if (agv_state == SYSTEM_STATE_NAVIGATING || agv_state == SYSTEM_STATE_FREE)
                {
                        // std::cout<<"?????????????????????????????????????????????...\n";
                        // if (p_map.is_nullptr())
                        // {
                                p_map = get_global_agv_instance()->get_current_map();
                                gridmap = p_map.get();
                        // }
                        if (p_map.is_nullptr())
                        {
                                std::cout << "????????????...\n";
                        }
                        agv_current_pose = get_global_agv_instance()->expolate_current_position();
                        //  printf("agv_current_pose:(%f,%f,%f)\n",agv_current_pose.x,agv_current_pose.y,agv_current_pose.theta);
                        agv_current_velocity = get_global_agv_instance()->logic_out_process->speed_data.vel_line;
                        printf("agv_current_velocity:%f\n",agv_current_velocity);
                        if (enable_speed_compensation && agv_current_velocity >= 1.0)
                                speed_compensation = 0.25; //?????????????????????1m/s????????????????????????????????????????????????????????????
                        else
                                speed_compensation = 0.;
                        sense_robot_environment(agv_current_pose, region_width_, region_length_, count_);
                        int time = agv_run_environment[0] + agv_run_environment[1] + agv_run_environment[2] + agv_run_environment[3];
                        printf("agv environment times:%d\n", time);
                        chose_agv_obstrategy(time);
                        get_agv_obavoid_state();
                        obavoid_state_output(ob_state);
                }
                if (agv_state == SYSTEM_STATE_MAPPING || agv_state == SYSTEM_STATE_MAPPING_LANDMARK)
                {
                        //std::cout<<"?????????????????????????????????????????????...\n";
                        portrait_adjust_level = 1;
                        transverse_adjust_level = 1;
                        get_agv_obavoid_state();
                        obavoid_state_output(ob_state);
                }
        }
        if(ptr_msg_queue != nullptr){
                Laser_obs_3d_data obs_3d;
                obs_3d.laser_obs_level_f_l = ob_state.ahead_obstacle;
                obs_3d.laser_obs_level_b_l = ob_state.rear_obstacle;
                obs_3d.laser_obs_level_l_l = ob_state.left_obstacle;
                obs_3d.laser_obs_level_r_l = ob_state.right_obstacle;
                std::vector<char> buf;
                obs_3d.to_char_array(&buf);
                ptr_msg_queue->send(obavoid_c,buf.data(),buf.size());
        }
        // std::cout<<"??????????????? \n"<<"?????? \t"<<ob_state.ahead_obstacle<<" ?????? \t"<< ob_state.rear_obstacle<<"\n"<<"?????? \t"<<ob_state.left_obstacle<<" ?????? \t"<< ob_state.right_obstacle<<"\n";
}

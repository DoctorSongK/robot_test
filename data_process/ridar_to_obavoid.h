#ifndef RIDAR_TO_OBAVOID_H_
#define RIDAR_TO_OBAVOID_H_
#include<iostream>
#include<assert.h>
#include<time.h>
#include<unistd.h>
#include<stdio.h>
#include<termios.h>
#include<memory>
#include<stdlib.h>
#include<pthread.h>
#include<vector>
#include<pthread.h>


#include "../common/simple_grid_map.h"
#include "../common/pose.h"
#include "../common/msg_queue.h"
#include "../common/sensor_data.h"
#include "../location/location.h"


#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/point_types.h>  //点类型相关定义
//#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
#include <pcl/filters/statistical_outlier_removal.h>  //滤波相关
#include <pcl/common/common.h>  

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <algorithm>
#define RADIAL_DIVIDER_ANGLE 0.18
/*
#include "../logic/agv.h"
#include "../common/configuration.h"
#include "../data_process/msg_queue.h"
#include "../locate_algotithm/location.h"
*/

enum agv_obavoid_strategy{normal,front_back_self_adaption,all_self_adaption};//正常避障，前后自适应避障，前后左右自适应避障（枚举从0开始）
enum agv_obavoid_region{front_rec,front_sector,front_back_rec,front_back_sector,rec_and_rec,rec_and_sector};//前矩形避障，前扇形避障，前后矩形避障，前后扇形避障，四周矩形避障，前后扇形左右矩形避障
enum agv_obavoid_value{emergency_braking=1,slow_down,warning,normal_run};

typedef struct{
        int ahead_obstacle;//前方障碍物
        int rear_obstacle;//后方障碍物
        int left_obstacle;//左侧障碍物
        int right_obstacle;//右侧障碍物
}obavoid_state;
 
// extern custom_point_cloud to_obavoid_data;
// extern obavoid_state agv_obavoid_state;

class AGV_Obavoid:public MsgQueueListener{
public:
        AGV_Obavoid(MsgQueue *msg_queue,int rec_channel,int send_channel);
        ~AGV_Obavoid();
        ///后期这边需要改成线程池内容
        void handle_agv_obavoid(PCL_PointCloudData point_data);
        void get_agv_obavoid_state();
        void init_obavoid();
        void obavoid_state_output(obavoid_state& agv_obavoid_state);
        void sense_robot_environment(Position agv_pose,float region_width,float region_length,int count);
        void chose_agv_obstrategy(int time);
        virtual void recieve_message(const int channel, char *buf, const int size);

        void pcl_point_cloud2custom_point_cloud(pcl::PointCloud<pcl::PointXYZI> msg_data);//这里将pcl点云数据格式转换成自定义点云数据格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr msg);//体素滤波
        pcl::PointCloud<pcl::PointXYZI>::Ptr statistical_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr msg);//统计滤波
        double get_distance(double x,double y);
        double get_distance(double x,double y,double z);//求点云距离
        vector<Raw_point> rectangular_to_polar(std::vector<PointXYZI> data);//直角坐标系转换成极坐标
        void XYZI_to_RTZColor();//其实这里的作用就是将所有的点云根据角度完成划分，然后将该角度内的点云序列再排序，另外这边根据半径得到的索引没有用到
        void remove_ground_points();


private:
        long pre_timestamp;
        std::list<pcl::PointCloud<pcl::PointXYZI>> recieved_raw_data;//用队列来接收传输过来的函数
        // std::vector<PointXYZI> raw_point_data;//收到的雷达的原始数据
        // std::vector<PointXYZI> filtered_point_data;
        // std::vector<PointXYZI> rm_ground_point_data;//去除地面之后的数据

        pcl::PointCloud<pcl::PointXYZI>::Ptr voxelgrid_output;
        pcl::PointCloud<pcl::PointXYZI>::Ptr statistical_output;
        std::vector<pcl::PointIndices> out_radial_divided_indices;
        std::vector<PointCloudXYZIRTColor> out_radial_ordered_clouds;
        pcl::PointIndices out_ground_indices;//剥离出的地面点索引
        pcl::PointIndices out_no_ground_indices;//剥离出的非地面点索引

        custom_point_cloud custom_data_3d;
        float max_height_pc;
        float range_min_;
        float range_max_;

        MsgQueue* ptr_msg_queue = nullptr;
        int rec_channel;//接收的原始避障点云通道
        int obavoid_c;//发送的避障通道
        bool enable_obavoid;
        bool enable_speed_compensation;//是否开启速度补偿
        agv_obavoid_strategy obavoid_strategy;
        agv_obavoid_region obavoid_region;
        agv_obavoid_value agv_obavoid_level;
        Position agv_current_pose;//获得机器人当前的位姿
        float agv_current_velocity;//获得机器人当前的速度
        float speed_compensation;//速度值给予避障距离的补偿量
        float avoid_dis1x;
        float avoid_dis2x;
        float avoid_dis3x;
        float avoid_dis1y;
        float avoid_dis2y;
        float avoid_dis3y;
        float avoid_lside;
        float avoid_rside;

        int colli_cntf1 = 0;
        int colli_cntf2 = 0;
        int colli_cntf3 = 0;
        int colli_cntl = 0;
        int colli_cntr = 0;
        int colli_cntb1 = 0;
        int colli_cntb2 = 0;
        int colli_cntb3 = 0;

        int collision_threshold;
        float begin_sector_angle;
        float end_sector_angle;
        float portrait_adjust_level;
        float transverse_adjust_level;
        vector<int> agv_run_environment;
        SimpleGridMap *gridmap = nullptr;
        float judge_threshold;//判断所处环境时候在狭窄环境下
        MapSharedPointer p_map;
        int rotate_time;
      //  pthread_t obavoid_thread;
        //static void* thread_function(void *param);

        ///雷达的安装位置
        float installation_position_x;//雷达相对于机器人中心的安装位置
        float installation_position_y;
        float installation_position_z;
        float robot_length;
        float robot_width;
        float region_width_;
        float region_length_;
        int count_;

        /*方法配置*/
        bool use_intensity;//是否使用激光能量值
        bool use_inf;//是否开始给点云值赋无穷大值
        bool use_voxel_filter;//是否应用体素率波
        bool use_statistical_filter;//是否应用统计滤波
        bool use_remove_ground;//是否需要地面点去除
        //bool use_clustering = true;//是否需要地面点去除

        float voxel_unit_size;//体素单元大小
        float k_nearest_neighbor_size;//统计滤波k最近邻个数
        float standard_deviation;//统计滤波标准差
        float radar_driver_angle;
        float concentric_divider_distance_;
        float local_max_slope_;//局部最大坡度
        float general_max_slope_;//一般最大坡度
        float min_height_threshold_;
        float reclass_distance_threshold_;
        size_t radial_dividers_num_;
        pthread_mutex_t agv_ob_3d;

};
#endif
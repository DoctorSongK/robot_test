#ifndef POINT_CLOUD_3D_DATA_H_
#define POINT_CLOUD_3D_DATA_H_
#include <vector>
#include <queue>
#include "math.h"
#include <iostream>
#include <pthread.h>
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
#include "../common/sensor_data.h"
#include "../device/radar3D.h"
#include "../common/msg_queue.h"
#include "../common/configuration.h"
#include <list>
#define radian_to _degree 180.0/M_PI
#define degree_to_radian M_PI/180.0
#define RADIAL_DIVIDER_ANGLE 0.18


      //std::vector<PointCloudXYZIRTColor> out_radial_ordered_clouds;
//     typedef struct{
//             std::vector<PointXYZI> point_cloud;
//             int width;
//             int height;
//             long timestamp;
//     }custom_point_cloud;

// extern pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud;
//extern Raw_2D_point_cloud to_mapping_data;
//extern custom_point_cloud to_obavoid_data;
//extern pthread_mutex_t radar_mutex;

class Point_cloud_3D_data:public MsgQueueListener{
public:
        Point_cloud_3D_data(MsgQueue *msg_queue,int rec_channel_,int send_channel_2d_);
        ~Point_cloud_3D_data();
        Raw_2D_point_cloud pcl_point_cloud2custom_point_cloud(pcl::PointCloud<pcl::PointXYZI> msg_data);//这里将pcl点云数据格式转换成自定义点云数据格式
        Raw_2D_point_cloud to_2d_point_cloud_data(custom_point_cloud data);//将多层点云融合处理得到单层点云
        long trans_timestamp(double timestamp);//将当前时间戳转换为cartographer所需的时间戳
        void filter_raw_point();//去除点云中nan点及远距离点，并删除不用的其他层的线，完成统计滤波，
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr msg);//体素滤波
        pcl::PointCloud<pcl::PointXYZI>::Ptr statistical_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr msg);//统计滤波
        double get_distance(double x,double y);
        double get_distance(double x,double y,double z);//求点云距离
        vector<Raw_point> rectangular_to_polar(std::vector<PointXYZI> data);//直角坐标系转换成极坐标
        void XYZI_to_RTZColor();//其实这里的作用就是将所有的点云根据角度完成划分，然后将该角度内的点云序列再排序，另外这边根据半径得到的索引没有用到
        void remove_ground_points();
        void handle_radar_data(PCL_PointCloudData data);//radar数据处理
        custom_point_cloud custom_data;//接收的自定义数据
        virtual void recieve_message(const int channel, char *buf, const int size);
private:
        MsgQueue * p_msg_queue = nullptr;
        int rec_channel;
        int send_channel_2d;
        int send_channel_ob;
        std::list<pcl::PointCloud<pcl::PointXYZI>> recieved_raw_data;//用队列来接收传输过来的函数
        //pthread_t recieve_3D_thread;
        //static void *recieve_data(void *param);
        std::vector<PointXYZI> raw_point_data;//收到的雷达的原始数据
        std::vector<PointXYZI> filtered_point_data;
        std::vector<PointXYZI> rm_ground_point_data;//去除地面之后的数据
        pcl::PointIndices out_ground_indices;//剥离出的地面点索引
        pcl::PointIndices out_no_ground_indices;//剥离出的非地面点索引
        std::vector<Raw_point> trans_data;//转换后得到的数据
        std::vector<std::vector<PointXYZI>> row_data;
        std::vector<std::vector<PointXYZI>> column_data;
        
        PointCloudXYZIRTColor out_organized_points;
        std::vector<pcl::PointIndices> out_radial_divided_indices;
        std::vector<PointCloudXYZIRTColor> out_radial_ordered_clouds;
        long pre_timestamp;
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxelgrid_output;
        pcl::PointCloud<pcl::PointXYZI>::Ptr statistical_output;

        int up_limit_floor; //这条线层数以下的数据不要
        float max_height_pc;//这个高度以上的点云不要
        float robot_height;//机器人的尺寸规模
        float robot_width;
        float robot_length;
        float installation_position_x;//雷达相对于机器人中心的安装位置
        float installation_position_y;
        float installation_position_z;
        float installation_position_theta;
        float max_obavoid_distance; //最大避障距离
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

        /*生成2D激光参数*/
        double min_height_;//融合形成平面点云的时候选取的最低高度点云
        double max_height_;//融合形成平面点云的时候选取的最高高度点云
        double angle_min_;//融合形成平面点云的最小角度
        double angle_max_;//融合形成平面点云的最大角度
        double angle_increment_;//角度增量，就是多大角度有一个激光点，根据设定的最小、最大角度及角度增量来确定平面点云的个数
        double scan_time_;//扫描时间
        double range_min_;//平面点云的最小距离；这里雷达输出的最小距离就是0.2m
        double range_max_;//平面点云的最大距离
        double inf_epsilon_;

       /*方法配置*/
        bool use_intensity;//是否使用激光能量值
        bool use_inf;//是否开始给点云值赋无穷大值
        bool use_voxel_filter;//是否应用体素率波
        bool use_statistical_filter;//是否应用统计滤波
        bool use_3D_to_2D_lidar;//是否需要将3D激光转换成2D激光
        bool use_remove_ground;//是否需要地面点去除
        //bool use_clustering = true;//是否需要地面点去除

        pthread_mutex_t to_2d_mutex;
       
};
#endif
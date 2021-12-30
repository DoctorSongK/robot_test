#ifndef _RADAR3D_H_ 
#define _RADAR3D_H_

//#define ROW 16
#define radar_working 1
#define radar_down 0
#define radar_monitor_working 1
#define radar_monitor_down 0
#include<iostream>
#include "rs_driver/api/lidar_driver.h"
#include "rs_driver/driver/driver_param.h"
#include <vector>
#include<string>
#include<pthread.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include"../common/time.h"
#include "../common/sensor_data.h"
#include "../common/msg_queue.h"
#include "../common/move.h"
////这个地方需要和小车程序进行对接
using namespace robosense::lidar;
using namespace std;



//extern pcl::PointCloud<pcl::PointXYZI> point_cloud;
// extern pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud;

class RadarModule3D{
    public:
            RadarModule3D();
            ~RadarModule3D();
            void close_radar();
            bool radar_state = radar_down;
            bool radar_monitor_state = radar_monitor_down;
            bool radar_disconnect = true;
            bool radar_error = false;
            static bool error_state;
            static int frequency;
            static int error_times;
            static int reconnect_time;
            static void pointCloudCallback(const PointCloudMsg<pcl::PointXYZI>& msg);
            static void exceptionCallback(const Error& code);
            static MsgQueue * p_msg_queue;
            static int ob_channel;
            static int map_channel;
            static void set_target(MsgQueue *target, int ob_channel_,int map_channel_);
    private:
            
            pthread_t radar;
            LidarDriver<pcl::PointXYZI> driver;  ///< Declare the driver object
            RSDriverParam driver_param;
            static void* radar_thread(void *param);
            static pthread_mutex_t radar_3d_mutex;
};




#endif
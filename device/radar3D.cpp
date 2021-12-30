#include "radar3D.h"
#include "../common/configuration.h"
#include"../logic/agv.h"

// pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud;
// pcl_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
bool RadarModule3D::error_state = false;
int RadarModule3D::frequency = 1;//静态成员变量需要在类外定义，在类内声明，一个变量
int RadarModule3D::error_times = 0;
int RadarModule3D::reconnect_time = 10000000;//10s循环检查一次
MsgQueue * RadarModule3D::p_msg_queue = nullptr;
int RadarModule3D::ob_channel = 0;
int RadarModule3D::map_channel = 0;
pthread_mutex_t RadarModule3D::radar_3d_mutex;
//point_cloud = *pcl_pointcloud;
// pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZI> point_cloud;

RadarModule3D::RadarModule3D(){
        /*radar常用参数配置*/
        //这边可以补充一下，雷达的通讯协议是udp形式，不会建立连接，所以可以不用开另一个线程来判断
        driver_param.input_param.device_ip = "192.168.192.200";
        driver_param.input_param.msop_port = 6699;
        driver_param.input_param.difop_port = 7788;
        driver_param.decoder_param.max_distance = 200.0f;
        driver_param.decoder_param.min_distance = 0.2f;
        driver_param.decoder_param.start_angle = 0.0f;
        driver_param.decoder_param.end_angle =360.0f;
        driver_param.frame_id = "rslidar";
        driver_param.lidar_type = LidarType::RS16;
        driver_param.saved_by_rows = true;//如果为true的话，雷达数据会以行的形式输出保存，默认为false，输出以列保存
        pthread_mutex_init(&RadarModule3D::radar_3d_mutex,nullptr);
        pthread_create(&radar,NULL,radar_thread,this);
}
RadarModule3D::~RadarModule3D(){
        close_radar();
        pthread_mutex_destroy(&RadarModule3D::radar_3d_mutex);
        pthread_join(radar,NULL);
}
void RadarModule3D::close_radar(){
        radar_state = radar_down;
}
void RadarModule3D::set_target(MsgQueue *target, int ob_channel_,int map_channel_){
        p_msg_queue = target;
        ob_channel = ob_channel_;
        map_channel = map_channel_;
}
void* RadarModule3D::radar_thread(void *param){
        RadarModule3D *ptr = (RadarModule3D *)param;
        int count = 10;
        while(count--){
                ptr-> driver.regExceptionCallback(ptr->RadarModule3D::exceptionCallback);  ///< Register the exception callback function into the driver
                ptr->driver.regRecvCallback(ptr->RadarModule3D::pointCloudCallback);   
                if (!ptr->driver.init(ptr->driver_param)){
                        cout << "Driver Initialize Error..." << endl;;
                        cout<<"Start Radar Reconnect..."<<endl;
                        ptr->radar_disconnect = true;
                        continue;
                }   
                else{
                        cout<<"radar init success"<<endl;
                        ptr->radar_disconnect = false;
                        break;
                }                
        }
        if(count == 0)
        {
                cout<<"radar init fail，please check radar device..."<<endl;
                exit(1);
        }
        ptr->driver.start();  ///< The driver thread will start
        ptr->radar_state = radar_working;
        ptr->radar_monitor_state = radar_monitor_working;
        cout << "radar start......" << endl;
        while (ptr->radar_state){
                //pthread_mutex_lock(&data_mutex);
               std::this_thread::sleep_for(std::chrono::seconds(1));
               ptr->radar_error = RadarModule3D::error_state;
              // usleep(10);
                //pthread_mutex_unlock(&data_mutex);
        }
}
                                                                
void RadarModule3D::pointCloudCallback(const PointCloudMsg<pcl::PointXYZI>& msg){
       
        if(RadarModule3D::frequency-- == 1){
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_ob(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_map(new pcl::PointCloud<pcl::PointXYZI>);

                pcl_pointcloud_ob->points.assign(msg.point_cloud_ptr->begin(),msg.point_cloud_ptr->end());
                pcl_pointcloud_map->points.assign(msg.point_cloud_ptr->begin()+7*msg.width,msg.point_cloud_ptr->begin()+9*msg.width);
               
                pcl_pointcloud_ob->height = msg.height;
                pcl_pointcloud_ob->width = msg.width;
                pcl_pointcloud_ob->is_dense = false;
                //pcl_pointcloud->header.stamp = get_current_time_us();
                pcl_pointcloud_ob->header.stamp = get_current_time_us();

                pcl_pointcloud_map->height = msg.height;
                pcl_pointcloud_map->width = msg.width;
                pcl_pointcloud_map->is_dense = true;
                //pcl_pointcloud->header.stamp = get_current_time_us();
                pcl_pointcloud_map->header.stamp = get_current_time_us();
               // std::cout<<"pcl_pointcloud_map.size  "<<pcl_pointcloud_map->points.size()<<std::endl;
               
                if(p_msg_queue != nullptr){
                        pthread_mutex_lock(&RadarModule3D::radar_3d_mutex);
                        PCL_PointCloudData Radar3D_to_ob(pcl_pointcloud_ob);
                        PCL_PointCloudData Radar3D_to_map(pcl_pointcloud_map);

                        std::vector<char> buf1;
                        std::vector<char> buf2;
                        Radar3D_to_ob.to_char_array(&buf1);
                        Radar3D_to_map.to_char_array(&buf2);
                        p_msg_queue->send(ob_channel, buf1.data(),buf1.size());
                        p_msg_queue->send(map_channel, buf2.data(),buf2.size());
                        pthread_mutex_unlock(&RadarModule3D::radar_3d_mutex);
                        }
                
                RadarModule3D::frequency = 1;
        }
        
}
void RadarModule3D::exceptionCallback(const Error& code){
     
      //  cout<<code.toString()<<endl;//返回雷达错误，这边是雷达厂家定义
        //pthread_mutex_lock(&radar_mutex);
        if(code.error_code!=0x00){
                cout<<code.toString()<<endl;
               RadarModule3D::error_state = true;
               RadarModule3D::error_times++;

               usleep(RadarModule3D::reconnect_time);
                //pthread_cond_wait(&re_cond,&radar_mutex);
        }
        else
             RadarModule3D::error_state = false;    
        if(RadarModule3D::error_times>20){
                cout<<"radar init fail，please check radar device..."<<endl;
                exit(1);
        }
        //pthread_mutex_unlock(&radar_mutex);
}

#include "point_cloud_3D_data.h"
#include "../common/time.h"

//雷达点云坐标系表示
//雷达上没有线连接的一侧为雷达y轴正方向，右侧为雷达x轴，z轴正方向为雷达正向放置时的正向
//雷达的坐标原点定义在雷达结构中心，高度距离底座39mm

//Raw_2D_point_cloud to_mapping_data;
//custom_point_cloud to_obavoid_data;
//FILE *fp;

Point_cloud_3D_data::Point_cloud_3D_data(MsgQueue *msg_queue,int rec_channel_,int send_channel_2d_)
{
        //raw_point_data = data;
        robot_height = get_configs()->get_float("agv","robot_height",nullptr);
        robot_width = get_configs()->get_float("agv","robot_width",nullptr);
        robot_length = get_configs()->get_float("agv","robot_length",nullptr);
        
        installation_position_x = get_configs()->get_float("radar","installation_position_x",nullptr);
        installation_position_y = get_configs()->get_float("radar","installation_position_y",nullptr);
        installation_position_z = get_configs()->get_float("radar","installation_position_z",nullptr); //这里的坐标关系可能有些不一样，下面一直用的是-installation_position_z
        installation_position_theta = get_configs()->get_float("radar","installation_position_theta",nullptr);
        max_obavoid_distance = 5.0;

        max_height_pc = 1.5;
        /*生成2D激光参数*/
        
        min_height_ = get_configs()->get_float("radar","min_height",nullptr);
        max_height_ = get_configs()->get_float("radar","max_height",nullptr);
        angle_min_ = get_configs()->get_float("radar","angle_min",nullptr);
        angle_max_ = get_configs()->get_float("radar","angle_max",nullptr);
        angle_increment_ = (get_configs()->get_float("radar","angle_increment",nullptr)) * degree_to_radian;
        scan_time_ = 0.1;                          //这里要和激光雷达的频率对应起来
        range_min_ = get_configs()->get_float("radar","range_min",nullptr);
        range_max_ = get_configs()->get_float("radar","range_max",nullptr);
        inf_epsilon_ = 1.0;
        radar_driver_angle = 0.18;
        concentric_divider_distance_ = 0.01;
        local_max_slope_ = 6;
        general_max_slope_ = 4;
        min_height_threshold_ = 0.05;
        reclass_distance_threshold_ = 0.2;
        radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);
        pre_timestamp = -10000000;
        
       /// use_intensity = (get_configs()->get_bool("radar","use_intensity",nullptr) == 1)?true:false;
       // use_inf = (get_configs()->get_bool("radar","use_inf",nullptr)==1)?true:false;
       /// use_voxel_filter = (get_configs()->get_bool("radar","use_voxel_filter",nullptr)== 1)?true:false;
       // use_statistical_filter = (get_configs()->get_bool("radar","use_statistical_filter",nullptr)== 1)?true:false;
       // use_3D_to_2D_lidar = (get_configs()->get_bool("radar","use_3D_to_2D_lidar",nullptr)== 1)?true:false;
      //  use_remove_ground = (get_configs()->get_bool("radar","use_remove_ground",nullptr)== 1)?true:false;
	use_intensity = true;
	use_inf = false;
	use_voxel_filter = false;
	use_statistical_filter = false;
	use_3D_to_2D_lidar = true;
	use_remove_ground = true;
        /*滤波参数*/
        voxel_unit_size = 0.15;         //体素单元大小0.05
        k_nearest_neighbor_size = 50.0; //统计滤波k最近邻个数
        standard_deviation = 1.0;       //统计滤波标准差
        // pthread_create(&recieve_3D_thread,NULL,recieve_data,this);
        this->p_msg_queue = msg_queue;
        rec_channel = rec_channel_;

        p_msg_queue->add_listener(rec_channel,this);
        send_channel_2d = send_channel_2d_;
        voxelgrid_output.reset(new pcl::PointCloud<pcl::PointXYZI>());
        statistical_output.reset(new pcl::PointCloud<pcl::PointXYZI>());
	//fp = fopen("ladar.txt","w");
        pthread_mutex_init(&to_2d_mutex,nullptr);
}
Point_cloud_3D_data::~Point_cloud_3D_data()
{
        // pthread_join(recieve_3D_thread,NULL);
        pthread_mutex_destroy(&to_2d_mutex);
}
// void *Point_cloud_3D_data::recieve_3D_thread(void *param){
//                 Point_cloud_3D_data *ptr = Point_cloud_3D_data *(param);

// }
void Point_cloud_3D_data::recieve_message(const int channel, char *buf, const int size)
{
        if (channel == CHANNEL_3D_TO_MAP_RAW)
        {
                PCL_PointCloudData data;
                data.from_char_array(buf, size);
                handle_radar_data(data);
        }
}
Raw_2D_point_cloud Point_cloud_3D_data::pcl_point_cloud2custom_point_cloud(pcl::PointCloud<pcl::PointXYZI> msg_data)
{
        /*知识补充*/
        //memset函数原型extern void *memset(void *buffer,int c,int count);其中buffer为指针或是数组，c是赋给buffer的值，count是buffer的长度
        //memcpy用来做内存拷贝，可以拿它拷贝任何数据类型的对象，可以指定拷贝的数据长度，而strcpy则只能拷贝字符串，遇到‘/0’就会结束拷贝
        //例子：char a[100],b[50];memcpy(b,a,sizeof(b))，这里把a中sizeof(b)个元素复制给b，注意如果应用sizeof(a),会造成b的内存地址溢出
        //char a[100],b[50];strcpy(a,b);如用strcpy(b,a),要注意a中的字符串长度（第一个'/0'之前）是否超过50位，如超过，则会造成b的内存地址溢出
        memset(&custom_data, 0, sizeof(custom_point_cloud));
        // custom_data.height = msg_data.height;
        // custom_data.width = msg_data.width;
        // custom_data.timestamp = msg_data.header.stamp;
        auto iter = msg_data.points.begin();
        Raw_2D_point_cloud point_2D;
        point_2D.timestamp = msg_data.header.stamp;
        int ranges_size = std::ceil(2*M_PI / angle_increment_);
        // 确定无障碍物数据的激光扫描光线的计算范围是否为无穷大或最大范围
        if (use_inf)
        {
                point_2D.ranges.assign(ranges_size, std::numeric_limits<double>::infinity()); //给这些点云全部赋值为无穷大
                point_2D.intensitys.assign(ranges_size, 0);
        }
        else
        {
                point_2D.ranges.assign(ranges_size, range_max_ + inf_epsilon_);
                point_2D.intensitys.assign(ranges_size, 0);
        }
        if(iter == msg_data.points.end()){
                point_2D.ranges.assign(ranges_size, 0);
                point_2D.intensitys.assign(ranges_size, 0);
        }

        for (; iter != msg_data.points.end(); iter++)
        {
               // fprintf(fp, "%f ,%f ,%f \n",iter->x,iter->y,iter->z);
                if (std::isnan(iter->x) || std::isnan(iter->y) || std::isnan(iter->z))
                        continue;
                if((iter->x <= robot_length/2-abs(installation_position_x)) && (iter->x >= -(robot_length/2+installation_position_x)) && (iter->y >= -robot_width/2) && (iter->y <= robot_width/2 ))//去除车体自身所占空间点云
                        continue; 
                if (iter->z > max_height_ || iter->z < min_height_)
                        continue;
                double range = get_distance(iter->x, iter->y);
                if (range < range_min_ || range > range_max_)
                        continue;
                double angle = atan2(iter->y, iter->x);
                // if (angle < angle_min_ || angle > angle_max_)
                //         continue;
                //对出现小范围的点进行更新
                if(angle < 0 )
                angle+=2*M_PI;
                int index = angle / angle_increment_; //如果新范围较小的话
                //fprintf(fp, "%f ,%f ,%f ,%d\n",iter->x,iter->y,iter->z,index);
                if (range < point_2D.ranges[index])
                {
                        point_2D.ranges[index] = range;
                        point_2D.intensitys[index] = iter->intensity/255;
                        if(point_2D.intensitys[index]>1.0)
                                point_2D.intensitys[index] = 1.0;
                }
        }
     
        return point_2D;
}

long Point_cloud_3D_data::trans_timestamp(double timestamp)
{
        return timestamp * 100000;
}

double Point_cloud_3D_data::get_distance(double x, double y, double z)
{
        return sqrt(x * x + y * y + z * z);
}

double Point_cloud_3D_data::get_distance(double x, double y)
{
        return sqrt(x * x + y * y);
}

vector<Raw_point> Point_cloud_3D_data::rectangular_to_polar(std::vector<PointXYZI> data)
{
        int i = 0;
        auto iter = data.begin();
        for (; iter != data.end(); iter++)
        {
                //trans_data[i].timestamp = trans_timestamp(iter->timestamp);
                trans_data[i].distance = get_distance(iter->x, iter->y, iter->z);
                trans_data[i].pitch = asin(iter->z / trans_data[i].distance);
                trans_data[i].yaw = atan(iter->x / iter->y) * 180 / M_PI;
        }
        return trans_data;
}

void Point_cloud_3D_data::handle_radar_data(PCL_PointCloudData data)
{
        //过程应该是先进行判断，首先判断数据帧是否是连续的
        //  std::cout<<pcl_pointcloud->points.size()<<std::endl;
        pcl::PointCloud<pcl::PointXYZI> new_msg;
        if ((data.pcl_pointcloud.header.stamp) == pre_timestamp)
        {
                return;
        }
        else
        {
                if (recieved_raw_data.empty())
                {
                        pre_timestamp = data.pcl_pointcloud.header.stamp;
                        new_msg = data.pcl_pointcloud;
                }
                else
                {
                        pre_timestamp = data.pcl_pointcloud.header.stamp;
                        recieved_raw_data.push_back(data.pcl_pointcloud);
                        new_msg = recieved_raw_data.back();
                        if(recieved_raw_data.size()>2)
                                recieved_raw_data.clear();
                }
        }
        pthread_mutex_lock(&to_2d_mutex);
        Raw_2D_point_cloud point_2D;
              point_2D = pcl_point_cloud2custom_point_cloud(new_msg);
        pthread_mutex_unlock(&to_2d_mutex);
     
         if (use_3D_to_2D_lidar)
        {
              
                PointCloudData pointcloud_2D(point_2D.timestamp); //接收得到的2D点云
                 int first_index = (int)abs((angle_min_)/angle_increment_);
                 int end_index = (int)abs((angle_max_)/angle_increment_);
		//std::cout<<"first_index: "<<first_index<<"end_index: "<<end_index<<std::endl;
                for (int i = first_index; i < point_2D.ranges.size(); i++)
                {
                        //坐标转换成以前的
                        //double x = (0. + sin((-M_PI_2 + angle_increment_ * i)) * point_2D.ranges[i]);
                       // double y = (0. + cos((-M_PI_2 + angle_increment_ * i)) * point_2D.ranges[i]);
                       // pointcloud_2D.add_point(x, y, point_2D.intensitys[i]);
			 double x = (0. + cos((0. + angle_increment_ * i)) * point_2D.ranges[i]);
                         double y = (0. + sin((0. + angle_increment_ * i)) * point_2D.ranges[i]);
                         pointcloud_2D.add_point(x, y, point_2D.intensitys[i]);

                }
		for (int j = 0; j <= end_index; j++)
                {
                        //坐标转换成以前的
                        //double x = (0. + sin((-M_PI_2 + angle_increment_ * i)) * point_2D.ranges[i]);
                       // double y = (0. + cos((-M_PI_2 + angle_increment_ * i)) * point_2D.ranges[i]);
                       // pointcloud_2D.add_point(x, y, point_2D.intensitys[i]);
			 double x = (0. + cos((0. + angle_increment_ * j)) * point_2D.ranges[j]);
                         double y = (0. + sin((0. + angle_increment_ * j)) * point_2D.ranges[j]);
                         pointcloud_2D.add_point(x, y, point_2D.intensitys[j]);

                }
                reverse(pointcloud_2D.points.begin(),pointcloud_2D.points.end());
                if (p_msg_queue != nullptr)
                {
                        std::vector<char> buf;
                        pointcloud_2D.to_char_array(&buf);
                        p_msg_queue->send(send_channel_2d, buf.data(), buf.size());
                }
        }
}

#include "sensor_data.h"

void ImuData2D::to_char_array(std::vector<char> *output)
{
    output->reserve(output->size() + sizeof(ImuData2D));
    char *p = (char *)this;
    for (int i = 0; i < sizeof(ImuData2D); i++)
    {
        output->push_back(p[i]);
    }
}

void ImuData2D::from_char_array(char *buf, int size)
{
    if (size >= sizeof(ImuData2D))
    {
        ImuData2D *p = (ImuData2D *)buf;
        timestamp_us = p->timestamp_us;
        linear_acceleration_x = p->linear_acceleration_x;
        linear_acceleration_y = p->linear_acceleration_y;
        angle_z = p->angle_z;
        angular_velocity_z = p->angular_velocity_z;
    }
}

void PCL_PointCloudData::to_char_array(std::vector<char> *output){
       //这里应该注意一下reserve和resize的用法
        //reserve的话是用于重新申请并改变当前vector对象的总空间大小
        //resize用于重新申请并改变当前vector对象的有效空间大小，同时已经完成赋值操作，如果接着push_back的话就会在后面的基础上接着拓展空间
		output->reserve(output->size()+sizeof(PCL_PointCloudData));
    	char *buf = (char*)this;
		for(int i = 0;i<sizeof(PCL_PointCloudData);i++){
            	output->push_back(buf[i]);
       	}
}
void PCL_PointCloudData::from_char_array(char *buf, int size){
        if(size>=sizeof(PCL_PointCloudData)){
            PCL_PointCloudData *p = (PCL_PointCloudData *)buf;
            pcl_pointcloud = p->pcl_pointcloud;
        }
}

void AGV_Obavoid_data::to_char_array(std::vector<char> *output){
       //这里应该注意一下reserve和resize的用法
        //reserve的话是用于重新申请并改变当前vector对象的总空间大小
        //resize用于重新申请并改变当前vector对象的有效空间大小，同时已经完成赋值操作，如果接着push_back的话就会在后面的基础上接着拓展空间
		output->reserve(output->size()+sizeof(AGV_Obavoid_data));
    	char *buf = (char*)this;
		for(int i = 0;i<sizeof(AGV_Obavoid_data);i++){
            	output->push_back(buf[i]);
       	}
}
void AGV_Obavoid_data::from_char_array(char *buf, int size){
        if(size>=sizeof(AGV_Obavoid_data)){
            AGV_Obavoid_data *p = (AGV_Obavoid_data *)buf;
            point_cloud = p->point_cloud;
            width = p->width;
            height = p->height;
            timestamp = p->timestamp;
        }
}

void PointCloudData::to_char_array(std::vector<char> *output)
{
    int size = sizeof(long) + sizeof(float) * points.size() * 3;
    char *buf = new char[size];
    char *p = buf;
    *((long *)p) = timestamp;
    p += sizeof(long);
    int num = points.size();
    for (int i = 0; i < num; i++)
    {
        *((float *)p) = (float)(points[i](0));
        p += sizeof(float);
        *((float *)p) = (float)(points[i](1));
        p += sizeof(float);
        *((float *)p) = (float)(intensities[i]);
        p += sizeof(float);
    }
    output->reserve(output->size() + size);
    for (int i = 0; i < size; i++)
    {
        output->push_back(buf[i]);
    }
}
void PointCloudData::from_char_array(char *buf, int size)
{
    if (size >= sizeof(long))
    {
        timestamp = *((long *)buf);
        char *p = buf + sizeof(long);
        size -= sizeof(long);
        points.clear();
        intensities.clear();
        int num = size / (sizeof(float) * 3);
        for (int i = 0; i < num; i++)
        {
            float x = *((float *)p);
            p += sizeof(float);
            float y = *((float *)p);
            p += sizeof(float);
            float in = *((float *)p);
            p += sizeof(float);
            add_point(x, y, in);
        }
    }
}

void WheelSensorData::to_char_array(std::vector<char> *output)
{
    int size = sizeof(long) + sizeof(int) + sizeof(long);
    output->reserve(output->size() + size);
    char *buf = new char[size];
    char *p = buf;
    *((long *)p) = timestamp;
    p += sizeof(long);
    *((int *)p) = type;
    p += sizeof(int);
    *((long *)p) = data;
    p += sizeof(long);
    for (int i = 0; i < size; i++)
    {
        output->push_back(buf[i]);
    }
}
void WheelSensorData::from_char_array(char *buf, int size)
{
    if (size >= sizeof(long) + sizeof(int) + sizeof(long))
    {
        char *p = buf;
        timestamp = *((long *)p);
        p += sizeof(long);
        type = *((int *)p);
        p += sizeof(int);
        data = *((long *)p);
        p += sizeof(long);
    }
}

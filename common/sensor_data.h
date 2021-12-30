#ifndef CARTO_SENSOR_DATA_H
#define CARTO_SENSOR_DATA_H

#include <vector>
/*pcl点云*/
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>

#include "pose.h"
#include "byte_data.h"


typedef struct
{
    float x;
    float y;
    float z;
    float intensity;
} PointXYZI;
typedef struct
{
    float x;
    float y;
    float intensity;
    double timestamp;
} PointXYIT;

typedef struct
{
    std::vector<PointXYZI> point_cloud;
    int width;
    int height;
    long timestamp;
} custom_point_cloud;

typedef struct
{
    long timestamp;
    float distance;
    float pitch;
    float yaw;
} Raw_point;

typedef struct
{
    long timestamp;
    std::vector<double> ranges;
    std::vector<float> intensitys;
} Raw_2D_point_cloud;

typedef struct
{
    PointXYZI point;
    float radius;
    float theta;
    size_t radial_div;
    size_t concentric_div;
    size_t original_index;
} PointXYZIRTColor;

typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

class ImuData2D : public Serializable
{
public:
    ImuData2D() : ImuData2D(0, 0., 0., 0.){};
    ImuData2D(long time_us, double angular_velocity) : ImuData2D(time_us, 0., 0., angular_velocity){};
    ImuData2D(long time_us,
              double acceleration_x,
              double acceleration_y,
              double angular_velocity)
        : timestamp_us(time_us),
          linear_acceleration_x(acceleration_x),
          linear_acceleration_y(acceleration_y),
          angular_velocity_z(angular_velocity){};

    long timestamp_us;
    double linear_acceleration_x;
    double linear_acceleration_y;
    double angular_velocity_z;
    double angle_z;

    virtual void to_char_array(std::vector<char> *output);
    virtual void from_char_array(char *buf, int size);
};

class PCL_PointCloudData : public Serializable
{
public:
    PCL_PointCloudData(){};
    PCL_PointCloudData(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr) : pcl_pointcloud(*ptr){};
    ~PCL_PointCloudData(){};
    pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
    virtual void to_char_array(std::vector<char> *output);
    virtual void from_char_array(char *buf, int size);
};

class AGV_Obavoid_data : public Serializable
{
public:
    AGV_Obavoid_data(){};
    AGV_Obavoid_data(custom_point_cloud data)
    {
        point_cloud.assign(data.point_cloud.begin(), data.point_cloud.end());
        width = data.width;
        height = data.height;
        timestamp = data.timestamp;
    };
    ~AGV_Obavoid_data(){};
    std::vector<PointXYZI> point_cloud;
    int width;
    int height;
    long timestamp;
    virtual void to_char_array(std::vector<char> *output);
    virtual void from_char_array(char *buf, int size);
};

class PointCloudData : public Serializable
{
public:
    PointCloudData() : PointCloudData(0){};
    PointCloudData(long time_us) : timestamp(time_us){};
    PointCloudData(long time_us, int size) : timestamp(time_us)
    {
        points.reserve(size);
        intensities.reserve(size);
    };

    void add_point(double x, double y)
    {
        add_point(x, y, 0.25);
    };
    void add_point(double x, double y, double intensity)
    {
        points.push_back(Eigen::Vector2d(x, y));
        intensities.push_back((float)intensity);
    };

    PointCloudData intercept(double intensity_threshold)
    {
        int size = points.size();
        PointCloudData ret(timestamp, size);
        for (int i = 0; i < size; i++)
        {
            if (intensities[i] > intensity_threshold)
            {
                ret.add_point(points[i](0), points[i](1), intensities[i]);
            }
        }
        return ret;
    }

    long timestamp;
    std::vector<Eigen::Vector2d> points;
    std::vector<float> intensities;

    virtual void to_char_array(std::vector<char> *output);
    virtual void from_char_array(char *buf, int size);
};

#define WHEEL_DATA_TYPE_LEFT 0
#define WHEEL_DATA_TYPE_RIGHT -1
#define WHEEL_DATA_TYPE_STEER_FV1 17
#define WHEEL_DATA_TYPE_STEER_FR1 18
#define WHEEL_DATA_TYPE_STEER_BV1 19
#define WHEEL_DATA_TYPE_STEER_BR1 20
/*********************************四舵轮*******************************************/
#define WHEEL_DATA_TYPE_STEER_LFV1 30 //左前行走
#define WHEEL_DATA_TYPE_STEER_LFR1 31 //左前转向
#define WHEEL_DATA_TYPE_STEER_RFV1 32 //右前行走
#define WHEEL_DATA_TYPE_STEER_RFR1 33 //右前转向
#define WHEEL_DATA_TYPE_STEER_LBV1 34 //左后行走
#define WHEEL_DATA_TYPE_STEER_LBR1 35 //左后转向
#define WHEEL_DATA_TYPE_STEER_RBV1 36 //右后行走
#define WHEEL_DATA_TYPE_STEER_RBR1 37 //右后转向
/*********************************************************************************/

#define ROBOT_DIFF_MODEL_CARRY 0
#define ROBOT_DIFF_MODEL_COMMON 1
#define ROBOT_DIFF_MODEL_COMMON_YB 2 ////迎宾机器人
#define ROBOT_DUAL_STEER_MODEL 5
/*********************************四舵轮*******************************************/
#define ROBOT_FOUR_STEER_MODEL 6
/*********************************************************************************/

class WheelSensorData : public Serializable
{
public:
    WheelSensorData() : WheelSensorData(0, -1, 0){};
    WheelSensorData(long time_us, int type, long data) : timestamp(time_us), type(type), data(data){};

    long timestamp;
    int type;
    long data;

    virtual void to_char_array(std::vector<char> *output);
    virtual void from_char_array(char *buf, int size);
};
#endif

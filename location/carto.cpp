#include <map>
#include <iostream>
#include <cmath>
#include <unistd.h>
//#include "../logic/agv.h"
#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/io/image.h>
#include <cartographer/io/submap_painter.h>

#include "../common/data_transform.h"
#include "../common/configuration.h"
#include "../common/time.h"
#include "../location/carto.h"
FILE *carto_odoraw_file;////未锟斤拷锟斤拷锟斤拷图锟斤拷锟斤拷锟侥硷拷

using namespace cartographer;

CartoModule::CartoModule(CartoModuleOption &opt) : option(opt), landmark_map(get_configs()->get_float("map", "landmark_map_resolution", nullptr)) {
    
    LOG(INFO) << "Initialize cartographer with configuration file " << option.config_files_dir << "/" << option.config_file_name;
    usleep(100000);
    intensity_threshold = get_configs()->get_float("radar", "intensity_threshold", nullptr);
    auto file_resolver = absl::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{option.config_files_dir});
    const std::string code = file_resolver->GetFileContentOrDie(option.config_file_name);
    common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    map_builder_options = mapping::CreateMapBuilderOptions(lua_parameter_dictionary.GetDictionary("map_builder").get());
    trajectory_builder_options = mapping::CreateTrajectoryBuilderOptions(lua_parameter_dictionary.GetDictionary("trajectory_builder").get());

	double radar_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
    double radar_y = get_configs()->get_float("radar", "radar_position_y", nullptr);
    double radar_theta = get_configs()->get_float("radar", "radar_position_theta", nullptr);
	radar_position.x = radar_x;
	radar_position.y = radar_y;
	radar_position.theta = radar_theta;
	
    //鍒涘缓MapBuilder
    map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(map_builder_options);
    //鍒涘缓trajectory
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> sensor_ids;
    sensor_ids.insert(SensorId{SensorType::RANGE, option.range_name});
    //sensor_ids.insert(SensorId{SensorType::IMU, option.imu_name});
    sensor_ids.insert(SensorId{SensorType::ODOMETRY, "odom0"});

    trajectory_id = map_builder->AddTrajectoryBuilder(sensor_ids, trajectory_builder_options,
        [this](const int id,
            const ::cartographer::common::Time time,
            const transform::Rigid3d local_pose,
            sensor::RangeData range_data_in_local,
            const std::unique_ptr<const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult> res)
        {
            OnLocalSlamResult2(id, time, local_pose, range_data_in_local);
        }
    );
    LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";
    //鑾峰彇trajectory_builder
    trajectory_builder = map_builder->GetTrajectoryBuilder(trajectory_id);
    if(!trajectory_builder) {
        LOG(ERROR) << "Get Trajectory Builder Failed";
    }

    pthread_mutex_init(&paint_mutex, nullptr);
    pthread_mutex_init(&sensor_mutex, nullptr);

    landmark_rate = get_configs()->get_float("landmark_scan_matching", "point_rate", nullptr);

    if(option.msg_queue != nullptr) {
        //imu_listener_id = option.msg_queue->add_listener(option.imu_channel, this);
        radar_listener_id = option.msg_queue->add_listener(option.radar_channel, this);
        odom_listener_id = option.msg_queue->add_listener(CHANNEL_ODOM, this);
    }
    odom_position.timestamp = get_current_time_us();
    skip_odometer_data = get_configs()->get_int("carto", "skip_odometer_data", nullptr);
	//carto_odoraw_file = fopen("carto_odoraw_file","w+");
};

CartoModule::~CartoModule() {
    stop_and_optimize();
    pthread_mutex_destroy(&paint_mutex);
    pthread_mutex_destroy(&sensor_mutex);

    for(auto &entry : landmarks) {
        delete entry.second;
    }
    if(option.msg_queue != nullptr) {
        //option.msg_queue->remove_listener(imu_listener_id);
        option.msg_queue->remove_listener(radar_listener_id);
        option.msg_queue->remove_listener(odom_listener_id);
    }
};

void CartoModule::recieve_message(const int channel, char *buf, const int size) {
    
    if(channel == option.radar_channel) {
        PointCloudData data(0);
        //if(odom_count++ > skip_odometer_data)
        {
        data.from_char_array(buf, size);
        handle_radar_data(data);
        //printf("insert carto radar point timestamp: %ld\n",data.timestamp);
        //odom_count = 0;
        }
    }
    else if(channel == CHANNEL_ODOM) {
        Position data;
        data.from_char_array(buf, size);
       // printf("insert carto odom timestamp: %ld\n",data.timestamp);
        odom_position = odom_position * data;
        //if(odom_count++ > skip_odometer_data)
        {
            Position tmp = odom_position * radar_position;
		//fprintf(carto_odoraw_file,"%ld %f %f %f\n",tmp.timestamp, tmp.x,tmp.y,tmp.theta);

		//tmp.timestamp = get_current_time_us();
		//tmp.theta = get_global_agv_instance()->obtain_imu_angle();
		//printf("aaaaaaaaa%f---%f---%f\n",tmp.x,tmp.y,tmp.theta);
		//tmp = get_global_agv_instance()->expolate_current_position();
            handle_odom_data(tmp);
            odom_count = 0;
        }
    }
    else if(channel == option.imu_channel) {
        ImuData2D data;
        data.from_char_array(buf, size);
       // printf("insert carto imu timestamp: %ld\n",data.timestamp_us);
        //handle_imu_data(data);
    }
}

int CartoModule::handle_odom_data(Position & data) {
    pthread_mutex_lock(&paint_mutex);
    if(trajectory_id < 0) {
        pthread_mutex_unlock(&paint_mutex);
        return 1;
    }
    pthread_mutex_lock(&sensor_mutex);

    if(latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.timestamp) 
    {
        sensor::OdometryData tmp{to_carto_time(data.timestamp), to_carto_ridig3d(data)};
        trajectory_builder->AddSensorData("odom0", tmp);
        latest_sensor_timestamp = data.timestamp;
    }

    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&paint_mutex);
    return 0;
};

int CartoModule::handle_radar_data(PointCloudData & data) {
    pthread_mutex_lock(&paint_mutex);
    if(trajectory_id < 0) {
        pthread_mutex_unlock(&paint_mutex);
        return 1;
    }
   // LOG(INFO)<<"Add range data at timestamp " << data.timestamp << " point count " << data.points.size();
    pthread_mutex_lock(&sensor_mutex);
    if(latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.timestamp) 
    {
        // printf("point_cloud is coming..........................................\n");
        trajectory_builder->AddSensorData(option.range_name, to_carto_point_cloud(data, option.radar_scan_time));
        latest_sensor_timestamp = data.timestamp;
        int size = data.points.size();
        if(size == data.intensities.size()) {
            std::vector<Eigen::Vector2d>* landmark = new std::vector<Eigen::Vector2d>();
            for(int i = 0;i < size;i++) {
                if(data.intensities[i] > intensity_threshold) {
                   // std::cout<<"data.intensities[i]"<<data.intensities[i]<<"intensity_threshold"<<intensity_threshold<<std::endl;
                    landmark->push_back(data.points[i]);
                }
            }
            landmarks.insert(std::pair<long, std::vector<Eigen::Vector2d>*>(data.timestamp, landmark));
        }
    }
    else{
         printf("timestamp is outdata....\n");
         printf("latest_sensor_timestamp %ld,data.timestamp %ld\n",latest_sensor_timestamp,data.timestamp);
    }
    
    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&paint_mutex);
    return 0;
};

int CartoModule::handle_imu_data(ImuData2D & data) {
    pthread_mutex_lock(&paint_mutex);
    if(trajectory_id < 0) {
        pthread_mutex_unlock(&paint_mutex);
        return 1;
    }
   // if(!option.using_imu) {
   //     pthread_mutex_unlock(&paint_mutex);
   //     return 1;
   // }
//    LOG(INFO)<<"Add imu data at timestamp " << data.timestamp_us << " " << data.linear_acceleration_x << " " << data.linear_acceleration_y;
    pthread_mutex_lock(&sensor_mutex);
    if(latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.timestamp_us) {
        trajectory_builder->AddSensorData(option.imu_name, to_carto_imu(data));
        latest_sensor_timestamp = data.timestamp_us;
    }
    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&paint_mutex);
    return 0;
};

void CartoModule::stop_and_optimize() {
    pthread_mutex_lock(&paint_mutex);
    if(trajectory_id < 0) {
        pthread_mutex_unlock(&paint_mutex);
        return;
    }
    //缁撴潫trajectory
    map_builder->FinishTrajectory(trajectory_id);
    trajectory_id = -1;
    usleep(1000000);
    //杩愯鏈€缁堢殑鍏ㄥ眬浼樺寲
    map_builder->pose_graph()->RunFinalOptimization();
    pthread_mutex_unlock(&paint_mutex);

};

void CartoModule::paint_map(std::vector<char> *output) {
    using io::PaintSubmapSlicesResult;
    using io::SubmapSlice;
    using mapping::SubmapId;
    
    MapInfo info;
    pthread_mutex_lock(&paint_mutex);
    double resolution = 0.05;

    //鑾峰彇鎵€鏈夊瓙鍥剧殑浣嶅Э
    std::map<SubmapId, SubmapSlice> submap_slices;
    auto submap_poses = map_builder->pose_graph()->GetAllSubmapPoses();
    for (const auto& submap_id_pose : submap_poses) {
        SubmapId submap_id = submap_id_pose.id;
        transform::Rigid3d pose = submap_id_pose.data.pose;
        int version = submap_id_pose.data.version;

        //鏌ヨ瀛愬浘鍐呭
        mapping::proto::SubmapQuery::Response response_proto;
        const std::string error = map_builder->SubmapToProto(submap_id, &response_proto);
        if (!error.empty()) {
            LOG(ERROR) << error;
            pthread_mutex_unlock(&paint_mutex);
            return ;
        }
        int query_version = response_proto.submap_version();
        if(response_proto.textures_size() == 0) {
            LOG(INFO) << "Responds of submap query is empty for submap '" << submap_id.submap_index << "'";
            continue;
        }
        //鎻愬彇绗竴涓猅exture
        auto first_texture = response_proto.textures().begin();
        std::string cells = first_texture->cells();
        int width = first_texture->width();
        int height = first_texture->height();
        resolution = first_texture->resolution();
		// LOG(INFO) << "############resolution=" << resolution<< "##############";
        transform::Rigid3d slice_pose = transform::ToRigid3(first_texture->slice_pose());
        auto pixels = io::UnpackTextureData(cells, width, height);

        //濉啓SubmapSlice
        SubmapSlice& submap_slice = submap_slices[submap_id];
        submap_slice.pose = pose;
        submap_slice.metadata_version = version;
        submap_slice.version = query_version;
        submap_slice.width = width;
        submap_slice.height = height;
        submap_slice.slice_pose = slice_pose;
        submap_slice.resolution = resolution;
        submap_slice.cairo_data.clear();
        submap_slice.surface = ::io::DrawTexture(
            pixels.intensity, pixels.alpha, width, height,
            &submap_slice.cairo_data
        );
    } //for (const auto& submap_id_pose : submap_poses)

    pthread_mutex_unlock(&paint_mutex);
    LOG(INFO) << "Get and draw " << submap_slices.size() << " submaps";

    //浣跨敤Submap缁樺埗鍦板浘
    auto painted_slices = PaintSubmapSlices(submap_slices, resolution);
    int width = cairo_image_surface_get_width(painted_slices.surface.get());
    int height = cairo_image_surface_get_height(painted_slices.surface.get());

    info.width = width;
    info.height = height;
    info.origen_x = -painted_slices.origin.x() * resolution + radar_position.x;
    info.origen_y = (-height + painted_slices.origin.y()) * resolution+ radar_position.y;
    info.resolution = resolution;

    SimpleGridMap *grid_map = new SimpleGridMap(info);

    uint32_t* pixel_data = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(painted_slices.surface.get()));
    grid_map->datas.reserve(width * height);
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value = observed == 0 ? -1 : common::RoundToInt((1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(100, value);
            grid_map->datas.push_back((char)value);
        }
    }

    grid_map->landmark = landmark_map.to_landmark_map();

    LOG(INFO) << "Paint map with width " << width << ", height " << height << ", resolution " << resolution << " landmarks " << grid_map->landmark.get_info().width << "x" << grid_map->landmark.get_info().height;
    
    grid_map->to_char_array(output);
    delete grid_map;

};

void CartoModule::OnLocalSlamResult(
    const int id, const common::Time time,
    const transform::Rigid3d local_pose,
    sensor::RangeData range_data_in_local,
    const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> insertion_result)
{
    OnLocalSlamResult2(id,time,local_pose,range_data_in_local);
    return;
};

void CartoModule::OnLocalSlamResult2(
    const int id, const common::Time time,
    const transform::Rigid3d local_pose,
    sensor::RangeData range_data_in_local)
{
    transform::Rigid3d local2global = map_builder->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
    transform::Rigid3d pose3d = local2global * local_pose;

    Position pose2d = from_carto_rigid3d(from_carto_time(time), pose3d);

	if(option.msg_queue != nullptr) {
		Position pose2d_agv = pose2d;
		pose2d_agv.x -= radar_position.x * cos(pose2d.theta) ;
        pose2d_agv.x += radar_position.y * sin(pose2d.theta);
		pose2d_agv.y -= radar_position.x * sin(pose2d.theta);
        pose2d_agv.y -= radar_position.y * cos(pose2d.theta);
		pose2d_agv.x += radar_position.x;
		std::vector<char> tmp;
		pose2d_agv.to_char_array(&tmp);
		option.msg_queue->send(option.pose_channel, tmp.data(), tmp.size());
	}

    auto search = landmarks.find(from_carto_time(time));
    int landmark_count = 0;
    if(search != landmarks.end()) {
        std::vector<Eigen::Vector2d>* points = search->second;
        for(Eigen::Vector2d &p : *points) {
            Position after_trans = pose2d * Position(0, p(0), p(1), 0.);
            landmark_map.add_point(after_trans.x, after_trans.y);
            landmark_count ++;
        }
        delete points;
        landmarks.erase(search);
    }

//    LOG(INFO) << "Local slam callback at " << from_carto_time(time) << " : " << pose3d.DebugString() << ". with landmark point " << landmark_count;

    return;
};

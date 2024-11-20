#include <opencv2/opencv.hpp>
#include <radar_system.h>
#include <ros/ros.h>

struct node_params {
    std::string config_path;
    oculii::ModeCommand mode;
} params;


void load_params(ros::NodeHandle& nh_){
    params.config_path = nh_.param<std::string>("config_path", "Invalid_Path");
    ROS_INFO("loading config: %s", params.config_path.c_str());
    int chs_mode = nh_.param<int>("mode", 1);
    ROS_INFO("Radar mode: %d", chs_mode);
    switch (chs_mode) {
        case 0:
            params.mode = oculii::ModeCommand::SENSOR_MODE_0;
            break;
        case 1:
            params.mode = oculii::ModeCommand::SENSOR_MODE_1;
            break;
        case 2:
            params.mode = oculii::ModeCommand::SENSOR_MODE_2;
            break;
        default:
            assert(false && "Invalid mode for Radar!");
            break;
    }
}

int main(int argc, char* argv[]){

    ros::init(argc,argv,"oculii_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    load_params(nh_);
    
    auto radar_mode = oculii::ModeCommand::SENSOR_MODE_2;
    auto handle = std::shared_ptr<oculii::RadarSystem>(oculii::RadarSystem::GetRadarSystemInstance(params.config_path));
    
    // start radar system
    oculii::RadarErrorCode status = handle->StartSystem();
    if (status == oculii::RadarErrorCode::SUCCESS)
        ROS_INFO("Start Radar System success! ");
    else {
        ROS_ERROR("Start Radar System fail with error: %s", oculii::ErrorToString(status).c_str());
        return 0;
    }

    std::vector<int> IDs{1};
    std::vector<oculii::ModeCommand> modes{radar_mode};
    handle->SendModeSwitchCmd(IDs, modes);

    status = handle->StartRadarReceive();
    if (status == oculii::RadarErrorCode::SUCCESS)
        ROS_INFO("Start Radar Receive success! ");
    else {
        ROS_ERROR("Start Radar Receive fail with error: %s", oculii::ErrorToString(status).c_str());
        return 0;
    }

    // get frame rate
    int radar_frame_rate = handle->GetFrameRate(1);
    ROS_INFO("Radar Device frame rate %d!", radar_frame_rate);

    ros::spin();
    return 0;
}
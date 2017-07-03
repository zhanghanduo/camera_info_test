#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H
#include <csignal>
#include <cstdio>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
class CCameraInfo
{
public:
	CCameraInfo(ros::NodeHandle nh);
	~CCameraInfo();
	ros::NodeHandle nh_;
//	camera_info_manager::CameraInfoManager cinfo_mgr;
	ros::Publisher pub_left_cam_info;
	ros::Publisher pub_right_cam_info;
    std::string left_frame_id;
    std::string right_frame_id;

    unsigned int seq1, seq2;
//	sensor_msgs::CameraInfoPtr cinfo_msg;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	image_transport::CameraPublisher camera_pub;
	sensor_msgs::CameraInfoPtr left_cam_info_msg, right_cam_info_msg;

	void imageCallback(const sensor_msgs::ImageConstPtr &image1, const sensor_msgs::ImageConstPtr &image2);
    void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t, unsigned int seq);
    void fillCamInfo(sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,
                                 std::string left_frame_id, std::string right_frame_id);
};







#endif
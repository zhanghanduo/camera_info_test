#include "camera_info.h"
using namespace std;
CCameraInfo::CCameraInfo(ros::NodeHandle nh):	nh_(nh), it(nh_)
{
    seq1 = 0;
    seq2 = 0;

    pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/wide/left/camera_info", 1); //left
    ROS_INFO("Advertized on topic /wide/left/camera_info");
    pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/wide/right/camera_info", 1); //right
    ROS_INFO("Advertized on topic /wide/right/camera_info");

    left_frame_id = "wide_camera";
    right_frame_id = left_frame_id;

}
CCameraInfo::~CCameraInfo()
{
	ROS_INFO("Destroying CameraInfo...");
}

void CCameraInfo::imageCallback(const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::ImageConstPtr &msg2) {

//	cinfo_msg->header = msg1->header;
//	camera_pub.publish(msg1, cinfo_msg);
    sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
    fillCamInfo(left_cam_info_msg, right_cam_info_msg, left_frame_id, right_frame_id);

    publishCamInfo(left_cam_info_msg, pub_left_cam_info, msg1->header.stamp, seq1);
    publishCamInfo(right_cam_info_msg, pub_right_cam_info, msg2->header.stamp, seq2);
    seq1++;  seq2++;
}
/* \brief Publish the informations of a camera with a ros Publisher
   * \param cam_info_msg : the information message to publish
   * \param pub_cam_info : the publisher object to use
   * \param t : the ros::Time to stamp the message
   */
void CCameraInfo::publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t, unsigned int seq) {
    cam_info_msg->header.stamp = t;
    cam_info_msg->header.seq = seq;
    pub_cam_info.publish(cam_info_msg);
}

/* \brief Get the information of the cameras and store them in an information message
 * \param left_cam_info_msg : the information message to fill with the left camera informations
 * \param right_cam_info_msg : the information message to fill with the right camera informations
 * \param left_frame_id : the id of the reference frame of the left camera
 * \param right_frame_id : the id of the reference frame of the right camera
 */
void CCameraInfo::fillCamInfo(sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,
                 string left_frame_id, string right_frame_id) {

    unsigned int width = 640;
    unsigned int height = 422;

//    float baseline = 0.2988; // baseline converted in meters

    float fx = 431.2;
    float fy = 429.2073;
    float cx = 311.5393;
    float cy = 219.8287;

    double k1 = -0.21762034;
    double k2 = 0.095076355;
    double k3 = -0.01901413;
    double p1 = 0;
    double p2 = 0;

    left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    left_cam_info_msg->D.resize(5);
    right_cam_info_msg->D.resize(5);
    left_cam_info_msg->D[0] = k1;
    left_cam_info_msg->D[1] = k2;
    left_cam_info_msg->D[2] = k3;
    left_cam_info_msg->D[3] = right_cam_info_msg->D[3] = p1;
    left_cam_info_msg->D[4] = right_cam_info_msg->D[4] = p2;
    right_cam_info_msg->D[0] = -0.227190546;
    right_cam_info_msg->D[1] = 0.1204271779;
    right_cam_info_msg->D[2] = -0.041400503;

    left_cam_info_msg->K.fill(0.0);
    right_cam_info_msg->K.fill(0.0);
    left_cam_info_msg->K[0] = fx;
    left_cam_info_msg->K[2] = cx;
    left_cam_info_msg->K[4] = fy;
    left_cam_info_msg->K[5] = cy;
    left_cam_info_msg->K[8] = right_cam_info_msg->K[8] = 1.0;
    right_cam_info_msg->K[0] = 430.5056;
    right_cam_info_msg->K[2] = 319.7618;
    right_cam_info_msg->K[4] = 428.3714;
    right_cam_info_msg->K[5] = 232.7190;

    left_cam_info_msg->R.fill(0.0);
    right_cam_info_msg->R.fill(0.0);
    left_cam_info_msg->R[0] = 0.9999953128007949;
    left_cam_info_msg->R[1] = -0.00148287992000623;
    left_cam_info_msg->R[2] = 0.002678701846664197;
    left_cam_info_msg->R[3] = 0.001474982239854864;
    left_cam_info_msg->R[4] = 0.9999945671502626;
    left_cam_info_msg->R[5] = 0.002947897106572601;
    left_cam_info_msg->R[6] = -0.002683058671105158;
    left_cam_info_msg->R[7] = -0.002943932251541919;
    left_cam_info_msg->R[8] = 0.9999920671980682;


    right_cam_info_msg->R[0] = 0.9998065789425078;
    right_cam_info_msg->R[1] = 0.003766079457743483;
    right_cam_info_msg->R[2] = -0.01930340251864649;
    right_cam_info_msg->R[3] = -0.00382293210773934;
    right_cam_info_msg->R[4] = 0.9999884609247781;
    right_cam_info_msg->R[5] = -0.002909159224377853;
    right_cam_info_msg->R[6] = 0.01929222365043856;
    right_cam_info_msg->R[7] = 0.002982392129001422;
    right_cam_info_msg->R[8] = 0.9998094395652656;

    left_cam_info_msg->P.fill(0.0);
    right_cam_info_msg->P.fill(0.0);
    left_cam_info_msg->P[0] = right_cam_info_msg->P[0] = 350.45051;
    left_cam_info_msg->P[2] = right_cam_info_msg->P[2] = 320.47257;
    left_cam_info_msg->P[5] = right_cam_info_msg->P[5] = 350.45051;
    left_cam_info_msg->P[6] = right_cam_info_msg->P[6] = 227.64654;
    left_cam_info_msg->P[10] = right_cam_info_msg->P[10] = 1.0;
    right_cam_info_msg->P[3] = -104737.030959;

    left_cam_info_msg->width = right_cam_info_msg->width = width;
    left_cam_info_msg->height = right_cam_info_msg->height = height;

    left_cam_info_msg->header.frame_id = left_frame_id;
    right_cam_info_msg->header.frame_id = right_frame_id;
}
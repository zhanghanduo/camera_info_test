#include <camera_info.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_info_test_node");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");


	//sensor_msgs::CameraInfo cinfo_msg = boost::make_shared<sensor_msgs::CameraInfo>(cinfo_mgr.getCameraInfo());
	CCameraInfo CCameraInfo(nh);

    message_filters::Subscriber<Image> image1_sub(nh, "/wide/left/image_raw", 1);
    message_filters::Subscriber<Image> image2_sub(nh, "/wide/right/image_raw", 1);

    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
    sync.registerCallback(boost::bind(&CCameraInfo::imageCallback, &CCameraInfo ,_1, _2));

	ros::spin();
	return 0;
}

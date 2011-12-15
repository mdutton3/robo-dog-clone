/*------------------------------------------------------------------*/

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <robo_dog/ballinfo.h>
#include <robo_dog/mode_change.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <sstream>
#include <cstdio>

#include <ros/ros.h>

namespace enc = sensor_msgs::image_encodings;

class GetZ {
	ros::NodeHandle nh_;
	ros::Subscriber xy_sub;
	ros::Subscriber depth_sub;
	ros::Publisher xyz_pub;

	sensor_msgs::ImageConstPtr lastDepthMsg;

public:
	GetZ() :
		nh_("~") {
		xy_sub = nh_.subscribe("xy", 1, &GetZ::OnXYMsg, this);
		depth_sub = nh_.subscribe("depth", 1, &GetZ::OnDepthMsg, this);

		xyz_pub = nh_.advertise<geometry_msgs::Point> ("xyz", 10);

		ROS_INFO("GetZ ROSINFO");
		fprintf(stderr,"GetZ stderr\n");
	}

	~GetZ() {
	}

	void OnXYMsg(const robo_dog::ballinfoConstPtr & msg) {
		//		printf("Got the XY\n");
		//		ROS_INFO("Info X: %f", msg->x);
		//		ROS_INFO("Info Y: %f", msg->y);
		int x = (int)msg->x;
		int y = (int)msg->y;

		ROS_INFO("X,Y = %d,%d", x, y);

		sensor_msgs::ImageConstPtr depthMsg = lastDepthMsg;

		if (!depthMsg.get())
			return;

		try {
			cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(depthMsg,
					enc::TYPE_32FC1);
			ROS_INFO("W,H = %d,%d", cvImage->image.cols, cvImage->image.rows);
			if( (x >= 0) && (x < cvImage->image.cols) && (y >= 0) && (y < cvImage->image.rows) )
			{
				ROS_INFO("Depth = %f", cvImage->image.at<float>(y, x) );
			}
		} catch (cv_bridge::Exception& e) {
			ROS_INFO("Error");
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void OnDepthMsg(const sensor_msgs::ImageConstPtr & msg) {
		lastDepthMsg = msg;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "GetZ");
	GetZ node;
	ros::spin();
	return 0;
}


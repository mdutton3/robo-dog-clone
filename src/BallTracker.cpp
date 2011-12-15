#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <robo_dog/ballinfo.h>
#include <robo_dog/mode_change.h>

#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class BallTracker {
	int consecuvite_counter;
	ros::NodeHandle nh_;
	CvScalar ball_hsv_min;
	CvScalar ball_hsv_max;
	CvScalar person_hsv_min;
	CvScalar person_hsv_max;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_sub_;

	ros::Subscriber mode_change;
	cv_bridge::CvImagePtr lastDepth;

	bool looking_for_ball;

	ros::Publisher ballinfo_pub;

	int message_counter;

	double lastx, lasty;

public:
	BallTracker() :
		nh_("~"), it_(nh_), lastx(0.0), lasty(0.0) {
		consecuvite_counter = 0;
		ball_hsv_min = cvScalar(20, 100, 90, 0);
		ball_hsv_max = cvScalar(45, 256, 256, 0);
		person_hsv_min = cvScalar(155, 100, 90, 0);
		person_hsv_max = cvScalar(175, 256, 256, 0);
		looking_for_ball = true;
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
				&BallTracker::findBall, this);
		depth_sub_ = it_.subscribe("/camera/depth/image", 1,
				&BallTracker::depth, this);

		mode_change = nh_.subscribe("mode_change", 1,
				&BallTracker::change_mode, this);

		ballinfo_pub = nh_.advertise<robo_dog::ballinfo> ("ballinfo", 10);

		cv::namedWindow(WINDOW);
	}

	~BallTracker() {
		cv::destroyWindow(WINDOW);
	}

	void change_mode(const robo_dog::mode_change::ConstPtr& msg) {
		if (msg->mode == 1) {
			looking_for_ball = false;
		} else {
			looking_for_ball = true;
		}

	}

	void depth(const sensor_msgs::ImageConstPtr& msg) {
		try {
			lastDepth = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
			if( lastDepth.get() )
			{
				unsigned int len = lastDepth->image.rows * lastDepth->image.cols;
				float * p = lastDepth->image.ptr<float>(0);
				for(unsigned int i = 0; i < len; ++i)
				{
					if( p[i] >= 2.0 )
						p[i] = 1.0;
					else if( p[i] > 0.0 )
						p[i] /= 2;
					else
						p[i] = 0.0;
				}
			}

		} catch (cv_bridge::Exception& e) {
			ROS_INFO("Error");
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void findBall(const sensor_msgs::ImageConstPtr& msg) {
		message_counter++;
		if (message_counter % 20 == 0) {
			return;
		}
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		unsigned char key;

		IplConvKernel * erodeSE = cvCreateStructuringElementEx(3, 3, 0, 0,
				CV_SHAPE_ELLIPSE, NULL);
		IplConvKernel * dilateSE = cvCreateStructuringElementEx(3, 3, 2, 2,
				CV_SHAPE_ELLIPSE, NULL);

		CvMemStorage* storage = cvCreateMemStorage(0); //needed for Hough circles


		CvSize size = cvSize(640, 480);
		IplImage * hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
		IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);
		IplImage* thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);

		// looking_for_ball is a bool, it needs to be defined somewhere...

		CvScalar hsv_min;
		CvScalar hsv_max;
		if (looking_for_ball) {
			hsv_min = ball_hsv_min;
			hsv_max = ball_hsv_max;
		} else { //looking for person
			hsv_min = person_hsv_min;
			hsv_max = person_hsv_max;
		}

		cvCvtColor(&static_cast<IplImage> (cv_ptr->image), hsv_frame,
				CV_BGR2HSV);
		// to handle color wrap-around, two halves are detected and combined
		cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);

		// Erode and the image to remove fine noise
		cvErode(thresholded, thresholded, erodeSE, 2);
		cvDilate(thresholded, thresholded, dilateSE, 6);

		// hough detector works better with some smoothing of the image
		cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 3, 3);
		CvSeq* circles =
				cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,
						thresholded->height / 4, 100, 40, 20, 200);

		CvMoments *moments = new CvMoments;
		cvMoments(thresholded, moments, 0);

		double x, y;

		double area = cvGetSpatialMoment(moments, 0, 0);
		x = cvGetSpatialMoment(moments, 1, 0) / area;
		y = cvGetSpatialMoment(moments, 0, 1) / area;
		ROS_INFO("Info (X %f , Y %f)", x, y);
		if (x == x) {//not NAN
			lastx = lastx * 0.50 + x * 0.50;
			lasty = lasty * 0.50 + y * 0.50;
		}

		cvCircle(&static_cast<IplImage> (cv_ptr->image), cvPoint(
				cvRound(x), cvRound(y)), 3, CV_RGB(0,255,0), -1, 8, 0);

		cvCircle(&static_cast<IplImage> (cv_ptr->image), cvPoint(
				cvRound(lastx), cvRound(lasty)), 3, CV_RGB(255,100,0), -1, 8, 0);

		// Publish the ball info
		if (x == x) {//not NAN
			ROS_INFO("consecuvite_counter %u", consecuvite_counter);
			if (consecuvite_counter >= 20) {
				robo_dog::ballinfo b;
				b.x = lastx;
				b.y = lasty;
				//ROS_INFO("Info (X %f , Y %f)", x, y);

				ballinfo_pub.publish(b);
			} else {
				consecuvite_counter++;
			}
		}

		else {
			ROS_INFO("NO Ball");
			consecuvite_counter = 0;
		}

		cv_bridge::CvImagePtr depthCopy = lastDepth;
		fprintf( stderr, "depth copy: %p\n", depthCopy.get() );
		ROS_INFO( "depth copy: %p\n", depthCopy.get() );
		if( depthCopy.get() )
			cvShowImage("depth", &static_cast<IplImage> (depthCopy->image));
		cvShowImage("frame", &static_cast<IplImage> (cv_ptr->image));
		cvShowImage("threshold", thresholded);
		key = cvWaitKey(10);
		cvReleaseStructuringElement(&erodeSE);
		cvReleaseStructuringElement(&dilateSE);
		cvReleaseImage(&hsv_frame);
		cvReleaseImage(&thresholded);
		cvReleaseImage(&thresholded2);
		cvReleaseMemStorage(&storage);
		delete moments;
		//ROS_INFO("Info X: %f", x);
		//ROS_INFO("Info Y: %f", y);


	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "BallTracker");
	BallTracker ic;
	ros::spin();
	return 0;
}


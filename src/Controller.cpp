#include <cstdlib>
#include <ctime>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"
#include "robo_dog/ballinfo.h"
#include "robo_dog/mode_change.h"
#include "tf/transform_datatypes.h"

enum State {
	STATE_WANDER_STRAIGHT,
	STATE_WANDER_TURN,
	STATE_TURN_TO_BALL,
	STATE_DRIVE_TO_BALL,
	STATE_WAIT
};

int state;
double ballPos = 0.0;
bool ballInSight = false;
bool obstacle = false;
time_t lastBallSight = 0;
double rotation = 0.0;
double rotationTarget = 0.0;
double rotationDirection = 1.0;

ros::Publisher drivePub;

void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
	obstacle = false;
	for(int i = 1; i < 5; i++) {
		if(msg->points[i].x < 0.4 && abs(msg->points[i].y) < 0.25) {
			obstacle = true;
		}
	}
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	rotation = tf::getYaw(msg->pose.pose.orientation);
}

void ballCallback(const robo_dog::ballinfo::ConstPtr& msg) {
	ballInSight = true;
	lastBallSight = clock();
	ballPos = (msg->x - 320.0) / 320.0;
	ROS_INFO("ball %f", msg->x);
}

void sendDriveCommand(double vel, double rotVel) {
	geometry_msgs::Twist msg;
	msg.linear.x = vel;
	msg.linear.y = 0.0;
	msg.linear.z = 0.0; 
	msg.angular.x = 0.0;
	msg.angular.y = 0.0;
	msg.angular.z = rotVel;
	drivePub.publish(msg);
}

double sign(double d) {
	return d >= 0.0 ? 1.0 : -1.0;
}

double normalizeRotation(double d) {
	while(d < -M_PI) {
		d += 2.0 * M_PI;
	}
	while(d > M_PI) {
		d -= 2.0 * M_PI;
	}
	return d;
}

int main(int argc, char **argv)
{
	srand(time(NULL));
	ros::init(argc, argv, "robodog");
	ros::NodeHandle n;
	drivePub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
	ros::Subscriber sonarSub = n.subscribe("/RosAria/sonar", 1, sonarCallback);
	ros::Subscriber odometrySub = n.subscribe("/RosAria/pose", 1, odometryCallback);
	ros::Subscriber ballSub = n.subscribe("ballinfo", 1, ballCallback);
	ros::Publisher ballModePub = n.advertise<robo_dog::mode_change>("/mode_change", 1);
	ros::Rate loopRate(10);

	state = STATE_WANDER_STRAIGHT;
	ROS_INFO("STATE_WANDER_STRAIGHT");


	while(ros::ok()) {
		if(difftime(clock(), lastBallSight) > 0.5) {
			ballInSight = false;
		}

		switch(state) {
		case STATE_WANDER_STRAIGHT:		
			sendDriveCommand(0.15, 0.0);

			if(obstacle) {
				state = STATE_WANDER_TURN;
				ROS_INFO("STATE_WANDER_TURN");
				rotationTarget = (double)rand() / (double)RAND_MAX * M_PI - M_PI / 2.0;
				rotationDirection = sign(rotationTarget);
				rotationTarget += rotationDirection * M_PI / 2.0;
				rotationTarget += rotation;
				rotationTarget = normalizeRotation(rotationTarget);
			}
			else if(ballInSight) {
				state = STATE_TURN_TO_BALL;
				ROS_INFO("STATE_TURN_TO_BALL");
			}
			break;

		case STATE_WANDER_TURN:
			sendDriveCommand(0.0, rotationDirection * 0.5);

			if(fabs(rotationTarget - rotation) < 0.1) {
				state = STATE_WANDER_STRAIGHT;
				ROS_INFO("STATE_WANDER_STRAIGHT");
			}
			else if(ballInSight) {
				state = STATE_TURN_TO_BALL;
				ROS_INFO("STATE_TURN_TO_BALL");
			}
			break;
		
		case STATE_TURN_TO_BALL:
			sendDriveCommand(0.0, ballPos * -0.3);
			if(fabs(ballPos) < 0.1) {
				state = STATE_DRIVE_TO_BALL;
				ROS_INFO("STATE_DRIVE_TO_BALL");
			}
			if(!ballInSight) {
				state = STATE_WANDER_STRAIGHT;
				ROS_INFO("STATE_WANDER_STRAIGHT");
			}
			break;
		
		case STATE_DRIVE_TO_BALL:
			sendDriveCommand(0.15, 0.0);
			if(fabs(ballPos) > 0.2) {
				state = STATE_TURN_TO_BALL;
				ROS_INFO("STATE_TURN_TO_BALL");
			}
			if(!ballInSight) {
				robo_dog::mode_change msg;
				msg.mode = 1;
				ballModePub.publish(msg);		
				state = STATE_WANDER_STRAIGHT;
				ROS_INFO("STATE_WANDER_STRAIGHT");
			}
			if(obstacle) {
				state = STATE_WAIT;
				ROS_INFO("STATE_WAIT");
			}
			
			break;

		case STATE_WAIT:
			sendDriveCommand(0.0, 0.0);
			break;
		}
		

		ros::spinOnce();
		loopRate.sleep();
	}

	sendDriveCommand(0.0, 0.0);
}

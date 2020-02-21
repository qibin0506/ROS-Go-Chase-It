#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& resp) {
	geometry_msgs::Twist cmd;
	cmd.linear.x = req.linear_x;
	cmd.angular.z = req.angular_z;
	
	ROS_INFO("subscibe num: %d", motor_command_publisher.getNumSubscribers());
	motor_command_publisher.publish(cmd);

	resp.msg_feedback = "publish linear.x: " + std::to_string(req.linear_x) + ", angular.z: " + std::to_string(req.angular_z);

	ROS_INFO_STREAM(resp.msg_feedback);
	return true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "drive_bot");
	ros::NodeHandle n;

	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::ServiceServer server = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

	ros::spin();

	return 0;
}

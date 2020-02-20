#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "sensor_msgs/Image.h"
#include <algorithm>

ros::ServiceClient client;

void drive_bot(float linear_x, float angular_z) {
	ball_chaser::DriveToTarget srv;

	srv.request.linear_x = linear_x;
	srv.request.angular_z = angular_z;

	if (client.call(srv)) {
		ROS_INFO("req params linear_x: %1.2f, angular_z: %1.2f", linear_x, angular_z);
	} else {
		ROS_ERROR("failed to call /ball_chaser/command_robot service");
	}
}

void process_image_callback(const sensor_msgs::Image image) {
	int white_pixel = 255;

	// TODO: Loop through each pixel in the image and check if there's a bright white one
    	// Then, identify if this pixel falls in the left, mid, or right side of the image
    	// Depending on the white ball position, call the drive_bot function and pass velocities to it
    	// Request a stop when there's no white ball seen by the camera

	float linear_x = 0.5;
	float angular_z = 0.5;

	int left_count = 0;
	int right_count = 0;
	int front_count = 0;

	int left_area = image.width / 3;
	int front_area = left_area * 2;

	for (int i = 0; i < image.height * image.step; i += 3) {
		int pixel_idx = i % (image.width * 3) / 3;
		int center = image.step / 2;

		int r = image.data[i];
		int g = image.data[i+1];
		int b = image.data[i+2];

		if (r != white_pixel || g != white_pixel || b != white_pixel) {
			continue;
		}
		
		if (pixel_idx <= left_area) {
			left_count++;		
		} else if (pixel_idx <= front_area) {
			front_count++;
		} else {
			right_count++;
		}
	}

	if (left_count == 0 && right_count == 0 && front_count == 0) {
		ROS_INFO("stop");
		drive_bot(0, 0);
	} else {
		std::vector<int> counts{left_count, right_count, front_count};
		int max = *std::max_element(counts.begin(), counts.end());

		ROS_INFO("max: %d", max);

		if (max == left_count) {
			// drive left
			drive_bot(0, -angular_z);
		} else if (max == right_count) {
			// drive right
			drive_bot(0, angular_z);
		} else {
			// drive front
			drive_bot(linear_x, 0);
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	ros::Subscriber sub = n.subscribe("camera/rgb/image_raw", 10, process_image_callback);
	
	ros::spin();

	return 0;
}

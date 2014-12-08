#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "vetexcanopen.h"

static bool _enabled = false;
static double _xscale = 1;
static double _yscale = 1;
static double _zscale = 1;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO("Received %f, %f, %f",
		       	msg->linear.x,
			msg->linear.y,
			msg->angular.z);

	if ( ! _enabled) {
		vetex_enable_movement();
		_enabled = true;
	}

	// Scale parameters are stored in the ros parameter server.
	// The scaling factor will be a multiplication that changes
	// velocity into a percentage from -100 to 100.
	// They are retrieved just before the spinonce in the main function.
	vetex_set_all_percentages(floor(msg->linear.x * _xscale),
				  floor(msg->linear.y * _yscale),
				  floor(msg->angular.z * _zscale));
}

void enableCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Received %d", msg->data);
	if (msg->data) {
		vetex_enable_movement();
	} else {
		vetex_disable_movement();
	}
}


int main (int argc, char **argv)
{
	// Initialize the vetex can stuff.
	vetex_initialize();

	ros::init(argc, argv, "vetex_driver");
	ros::NodeHandle n("~");
	ros::Subscriber sub1 = n.subscribe("vetexvels", 1000, twistCallback);
	ros::Subscriber sub2 = n.subscribe("vetexenable", 1000, enableCallback);

	while (ros::ok()) {
		n.param("vetex_x_scale", _xscale, 1.0);
		n.param("vetex_y_scale", _yscale, 1.0);
		n.param("vetex_z_scale", _zscale, 1.0);
		ros::spinOnce();
	}

	// Terminate the vetex can stuff.
	vetex_terminate();
}

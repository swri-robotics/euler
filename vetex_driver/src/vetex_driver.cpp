#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "vetexcanopen.h"

/*
static bool enabled_ = false;
static double _xscale = 1;
static double _yscale = 1;
static double _zscale = 1;
*/

static const double SCALE_MAX_X = 100.0f;
static const double SCALE_MIN_X = 20.0f;
static const double SCALE_MAX_Y = 100.0f;
static const double SCALE_MIN_Y = 20.0f;
static const double SCALE_MAX_R = 100.0f;
static const double SCALE_MIN_R = 20.0f; 
static const double THROTLE_SCALE_FACTOR = 40.0f; 

class VetexDriver
{
public:
  VetexDriver():
    enabled_(false),
    _xscale(1),
    _yscale(1),
    _zscale(1),
    throtle_scale_factor_(THROTLE_SCALE_FACTOR)    
  {
    
  }

  ~VetexDriver()
  {

  }

  bool run()
  {
    ros::NodeHandle ph("~");
    ros::NodeHandle nh;

	  twist_sub_ = nh.subscribe("vetexvels", 1000, &VetexDriver::twistCallback,this);
	  enable_sub_ = nh.subscribe("vetexenable", 1000, &VetexDriver::enableCallback,this);


    if(init())
    {
      ROS_INFO_STREAM("Vetex driver initialized");
    }
    else
    {
      ROS_ERROR_STREAM("Vetex driver failed to initialize, aborting");
      vetex_terminate();
      return false;      
    }


    ros::Duration loop_duration(0.1f);
	  while (ros::ok())
    {
		  ph.param("vetex_x_scale", _xscale, 1.0);
		  ph.param("vetex_y_scale", _yscale, 1.0);
		  ph.param("vetex_z_scale", _zscale, 1.0);
		  ros::spinOnce();

      loop_duration.sleep();

	  }
    
	  vetex_terminate();

    return true;
  }

protected:

  bool init()
  {
	  // Initialize the vetex can stuff.
	  vetex_initialize();
    ros::NodeHandle n("~");

	  n.param("max_speed_x", max_speed_x_, 0.1); // m/sec
	  n.param("min_speed_x", min_speed_x_, 0.01);// m/sec
	  n.param("max_speed_y", max_speed_y_, 0.1); // m/sec
	  n.param("min_speed_y", min_speed_y_, 0.01);// m/sec
	  n.param("max_speed_r", max_speed_r_, 0.5); // m/sec
	  n.param("min_speed_r", min_speed_r_, 0.05);// m/sec

    return true;

  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
	  ROS_INFO("Received %f, %f, %f",
		         	msg->linear.x,
			  msg->linear.y,
			  msg->angular.z);

	  if ( ! enabled_) {
		  vetex_enable_movement();
      vetex_set_all_percentages(0, 0, 0);
		  enabled_ = true;
      ROS_INFO_STREAM("Movement enabled");
	  }

	  // Scale parameters are stored in the ros parameter server.
	  // The scaling factor will be a multiplication that changes
	  // velocity into a percentage from -100 to 100.
	  // They are retrieved just before the spinonce in the main function.

    double xf = computeScaleFactor(SCALE_MAX_X,SCALE_MIN_X, max_speed_x_,min_speed_x_,msg->linear.x);
    double yf = computeScaleFactor(SCALE_MAX_Y,SCALE_MIN_Y, max_speed_y_,min_speed_y_,msg->linear.y);
    double rf = -computeScaleFactor(SCALE_MAX_R,SCALE_MIN_R, max_speed_r_,min_speed_r_, msg->angular.z); // rotation is flipped

    ROS_INFO_STREAM("Scale factors x: "<<xf<<" y: "<<yf<<" r: "<<rf);

	  vetex_set_all_percentages(xf, yf, rf);

/*
	  vetex_set_all_percentages(floor(msg->linear.x * _xscale),
				    floor(msg->linear.y * _yscale),
				    floor(msg->angular.z * _zscale));

*/
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

  double computeScaleFactor(const double &max_factor,const double &min_factor,
    const double &max_speed,const double &min_speed,
    const double &command_speed)
  {
    double af = std::abs(command_speed);
    int sign = command_speed < 0 ? -1 : 1;
    if(af < min_speed)
    {
      af = 0; 
    }
    else if(af > max_speed)
    {
      af = throtle_scale_factor_;
    }
    else
    {
      af = max_factor - ((max_factor - min_factor)*(max_speed - af))/(max_speed - min_speed);
      af = (af > throtle_scale_factor_) ? throtle_scale_factor_ : af;
    }  

    return af*sign;
  }


protected:

  ros::Subscriber twist_sub_;
  ros::Subscriber enable_sub_;


  bool enabled_;
  double _xscale;
  double _yscale;
  double _zscale;

  double max_speed_x_;
  double min_speed_x_;
  double max_speed_y_;
  double min_speed_y_;
  double max_speed_r_;
  double min_speed_r_;
  double throtle_scale_factor_;

};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "vetex_driver");
  ros::NodeHandle nh;
  VetexDriver vd;
  if(vd.run())
  {
    return 0;
  }
  else
  {
    return -1;
  }

}

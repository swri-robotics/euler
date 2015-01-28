#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "vetexcanopen.h"
#include <boost/thread.hpp>

static const double SCALE_MAX_X = 100.0f;
static const double SCALE_MIN_X = 20.0f;
static const double SCALE_MAX_Y = 100.0f;
static const double SCALE_MIN_Y = 20.0f;
static const double SCALE_MAX_R = 100.0f;
static const double SCALE_MIN_R = 20.0f; 
static const double THROTLE_SCALE_FACTOR = 40.0f; 
static const double RAMP_TIME = 0.25f;
static const int RAMP_INCREMENTS = 10;
static const double STOP_RAMP_TIME = 0.25f;
static const int STOP_RAMP_INCREMENTS = 10; 


class VetexDriver
{
public:
  VetexDriver():
    enabled_(false),
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

	  twist_sub_ = nh.subscribe("vetexvels", 1, &VetexDriver::twistCallback,this);
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
    setEnable(true);
	  while (ros::ok())
    {
		  ros::spinOnce();
      loop_duration.sleep();
	  }
    
    setEnable(false);
    ros::Duration(1.0f).sleep();
	  vetex_terminate();

    return true;
  }

protected:

  bool init()
  {
	  // Initialize the vetex can stuff.	  
    ros::NodeHandle n("~");

	  n.param("max_speed_x", max_speed_x_, 1.0); // m/sec
	  n.param("min_speed_x", min_speed_x_, 0.01);// m/sec
	  n.param("max_speed_y", max_speed_y_, 1.0); // m/sec
	  n.param("min_speed_y", min_speed_y_, 0.01);// m/sec
	  n.param("max_speed_r", max_speed_r_, 0.5); // m/sec
	  n.param("min_speed_r", min_speed_r_, 0.05);// m/sec
    n.param("throtle_scale_factor", throtle_scale_factor_, THROTLE_SCALE_FACTOR);// m/sec
    n.param("ramp_time",ramp_time_,RAMP_TIME); // secs
    n.param("ramp_increments",num_ramp_increments_,RAMP_INCREMENTS); // int
    

    ROS_INFO_STREAM("Driver Parameters :\n"<<
      "\n\tmax_speed_x: "<<max_speed_x_<<
      "\n\tmax_speed_y: "<<max_speed_y_<<
      "\n\tmax_speed_r: "<<max_speed_r_<<
      "\n\tmin_speed_x: "<<min_speed_x_<<
      "\n\tmin_speed_y: "<<min_speed_y_<<
      "\n\tmin_speed_r: "<<min_speed_r_<<
      "\n\tramp_time: "<<ramp_time_<<
      "\n\tramp_increments: "<<num_ramp_increments_<<
      "\n\tthrotle_scale: "<<throtle_scale_factor_<<"\n");
      
    current_twist_.linear.x = 0;
    current_twist_.linear.y = 0;
    current_twist_.angular.z = 0;
    
    vetex_initialize();
    
    //signal(SIGINT,&VetexDriver::stopOnShutdown,this);

    return true;
  }

  void setEnable(bool enable)
  {
  
    geometry_msgs::Twist target_twist;
    if(enable)
    {      
      vetex_enable_movement();         
      ros::Duration(0.5f).sleep();   
      vetex_set_all_percentages(0, 0, 0);
      current_twist_.linear.x = 0;
      current_twist_.linear.y = 0;
      current_twist_.angular.z = 0;
      
      ROS_INFO_STREAM("Movement enabled");
      enabled_ = true;
    }
    else
    {
    
      target_twist.linear.x = 0;
      target_twist.linear.y = 0;
      target_twist.angular.z = 0;
    
      rampToSpeed(target_twist,STOP_RAMP_TIME,STOP_RAMP_INCREMENTS);    
      vetex_set_all_percentages(0, 0, 0);
      ros::Duration(0.2f).sleep(); 
      vetex_disable_movement();
      
      ROS_INFO_STREAM("Movement disabled");   
      enabled_ = false;   
    }

  }
  
  void stopOnShutdown(int sig)
  {
    ROS_WARN_STREAM("Vetex stopOnShutdown(...) invoked");  
  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
	  ROS_DEBUG("Received %f, %f, %f",msg->linear.x, msg->linear.y,msg->angular.z);

	  if ( ! enabled_) 
    {
      ROS_WARN_STREAM("Movement is currently disabled");
	  }

	  // Scale parameters are stored in the ros parameter server.
	  // The scaling factor will be a multiplication that changes
	  // velocity into a percentage from -100 to 100.
	  // They are retrieved just before the spinonce in the main function.

    //xf,yf, rf - are base velocities in the base coordiate system (as specified in the URDF)
    double xf = computeScaleFactor(SCALE_MAX_X,SCALE_MIN_X, max_speed_x_,min_speed_x_,msg->linear.x);
    double yf = computeScaleFactor(SCALE_MAX_Y,SCALE_MIN_Y, max_speed_y_,min_speed_y_,msg->linear.y);
    double rf = computeScaleFactor(SCALE_MAX_R,SCALE_MIN_R, max_speed_r_,min_speed_r_, msg->angular.z); // rotation is flipped
    ROS_DEBUG_STREAM("Scale factors(in URDF frame) x: "<<xf<<" y: "<<yf<<" r: "<<rf);

    // rxf, ryf, rrf - are reoriented base velocites as assumed by the controller(note, 
    // the rotation orientation is flipped because the controller is not a
    // right-handed frame - this is probably due to mimicing the hand control outputs which
    // aren't really a coordinate frame)
    double rxf = yf;
    double ryf = -xf;
    double rrf = -rf;
    ROS_WARN_STREAM("Received twist "<<msg->linear.x <<", "<<msg->linear.y <<", "<<msg->angular.z );
    ROS_WARN_STREAM("Reoriented factors(in base controller 'frame') x: "<<rxf<<" y: "<<ryf<<" r: "<<rrf);
    
    geometry_msgs::Twist target_twist;
    target_twist.linear.x = rxf;
    target_twist.linear.y = ryf;
    target_twist.angular.z = rrf;
    
    rampToSpeed(target_twist,ramp_time_,num_ramp_increments_);

	  //vetex_set_all_percentages(rxf, ryf, rrf);
  }
  
    
  bool rampToSpeed(const geometry_msgs::Twist& target_twist, double ramp_time, int num_increments)
  {
    std::vector<double> speeds(3,0);
    std::vector<double> increments(3,0);
    bool ramping_up = true;
    ros::Duration ramp_pause = ros::Duration(ramp_time/num_increments);

    increments[0] = (target_twist.linear.x - current_twist_.linear.x)/num_increments;
    increments[1] = (target_twist.linear.y - current_twist_.linear.y)/num_increments;
    increments[2] = (target_twist.angular.z - current_twist_.angular.z)/num_increments;

    ROS_WARN_STREAM("Ramping to twist "<<target_twist.linear.x <<", "<<target_twist.linear.y <<", "<<target_twist.angular.z );
    for(int i = 1;i <= num_increments;i++)
    {    
      speeds[0] = current_twist_.linear.x + increments[0]*i;
      speeds[1] = current_twist_.linear.y + increments[1]*i;
      speeds[2] = current_twist_.angular.z + increments[2]*i;
      
      vetex_set_all_percentages(speeds[0], speeds[1], speeds[2]);
      
      ramp_pause.sleep();
    } 
    
    ROS_WARN_STREAM("Reached goal twist with twist "<<speeds[0]<<" y: "<<speeds[1]<<" r: "<<speeds[2]);
    
    // saving current twist
    current_twist_ = target_twist;    
  
    return true;
  }

  void enableCallback(const std_msgs::Bool::ConstPtr& msg)
  {
	  ROS_INFO("Received %d", msg->data);
    setEnable(msg->data);
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
  double max_speed_x_;
  double min_speed_x_;
  double max_speed_y_;
  double min_speed_y_;
  double max_speed_r_;
  double min_speed_r_;
  double throtle_scale_factor_;
  double ramp_time_;
  int num_ramp_increments_;
  
  geometry_msgs::Twist current_twist_;

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

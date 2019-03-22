#include <ros/ros.h>
#include <iiwa_ros/iiwa_ros.h>


static int FRI_PORT = 30200;
static std::string FRI_HOSTNAME= "172.31.1.147";
static int FRI_TIMEOUT = 500.0;

int main(int argc, char* argv[] )
{
  ros::init( argc, argv, "test_fri");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  iiwa_ros::iiwaRos my_iiwa;
  Eigen::VectorXd wrench_filt_dead_band(6); wrench_filt_dead_band << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5;
  Eigen::VectorXd twist_filt_dead_band(6);  twist_filt_dead_band << 0.005, 0.005, 0.005, 0.005, 0.005, 0.005;
  
  double dt = 0.005;
  my_iiwa.init( dt, true );
      
  my_iiwa.getServoMotion().setFRIJointPositionControlMode(  );
  my_iiwa.startFriPublisher();

  ros::Time st = ros::Time::now();
  bool compensated = false;
  while(ros::ok())
  {
    if( (ros::Time::now() - st).toSec() > 20 )
    {
      ROS_INFO("Press Enter to compensate the weight");
      std::cin.get();
      my_iiwa.setWrenchOffset(2);
      compensated = true;
    }
    ros::Duration(0.5).sleep();
  }
    
  return -1;
}

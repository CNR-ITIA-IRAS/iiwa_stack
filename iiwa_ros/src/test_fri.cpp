#include <ros/ros.h>
#include <iiwa_ros/iiwa_ros.h>


static int FRI_PORT = 30200;
static std::string FRI_HOSTNAME= "172.31.1.147";
static int FRI_TIMEOUT = 500.0;

int main(int argc, char* argv[] )
{
  ros::init( argc, argv, "test_fri");
  ros::NodeHandle nh("~");

  iiwa_ros::iiwaRos my_iiwa;
  Eigen::VectorXd wrench_filt_dead_band(6); wrench_filt_dead_band << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5;
  Eigen::VectorXd twist_filt_dead_band(6);  twist_filt_dead_band << 0.005, 0.005, 0.005, 0.005, 0.005, 0.005;
  
  double dt = 0.005;
  my_iiwa.init( dt, 20.0, wrench_filt_dead_band, 20, twist_filt_dead_band, true );
  
    
  my_iiwa.getServoMotion().setFRIJointPositionControlMode(  FRI_PORT
                                                          , FRI_HOSTNAME
                                                          , FRI_TIMEOUT
                                                          , "/olivia/robot_description"
                                                          , "olivia_link_0"
                                                          , "olivia_link_ee" );
  
    
    ros::waitForShutdown();
    
  return -1;
}

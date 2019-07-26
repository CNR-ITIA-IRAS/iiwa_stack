#pragma once

#ifdef ENABLE_FRI

#include <thread>
#include <string>
#include <mutex>

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/CartesianQuantity.h>

#include <iiwa_fri/friUdpConnection.h>
#include <iiwa_fri/friClientApplication.h>
#include <iiwa_fri/friLBRClient.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Core>

#include <realtime_utilities/circular_buffer.h>
#include <eigen_state_space_systems/eigen_common_filters.h>

#include <iiwa_ros/common.h>

inline double timer_difference_s( struct timespec *timeA_p, struct timespec *timeB_p )
{
    double ret = ( ( ( double ) ( timeA_p->tv_sec ) + ( (double) timeA_p->tv_nsec)/1.e9 )
                -( ( double ) ( timeB_p->tv_sec ) + ( (double) timeB_p->tv_nsec)/1.e9 ) );
    return ret;
}


 
template< typename T > std::string to_string  ( const std::vector< T >& v
                                              , const std::string prefix = "["
                                              , const std::string delimeter = ", "
                                              , const std::string terminator ="]" )
{
  std::string ret = prefix;
  if(v.size() == 0)
    return "";
  
  for( size_t i=0; i < v.size()-1; i++)
    ret += std::to_string( v[i] ) + delimeter;
  ret += std::to_string( v.back() ) + terminator;
  
  return ret;
}

inline double diff  ( const std::vector< double >& v1
                    , const std::vector< double >& v2 )
{
  assert(v1.size()==v2.size() );
  assert(v1.size()>0);
  
  std::vector < double >  diff;
  
  for( size_t i=0; i< v1.size(); i++ )
      diff.push_back(std::fabs( v1[i] - v2[i] ) );
  
  return *std::max_element( diff.begin(), diff.end() );
}


/**
 * 
 */
struct Filter
{
  bool initialized_;
  Eigen::VectorXd values_;
  Eigen::VectorXd dead_band_;
  Eigen::VectorXd saturation_; 
  double natural_frequency_;
  double sampling_time_;
  std::vector < eigen_control_toolbox::FirstOrderLowPass *> lpf;

  Filter( ) : initialized_(false ){ }

  void init( const Eigen::VectorXd& dead_band
           , const Eigen::VectorXd& saturation
           , const double natural_frequency
           , const double sampling_time
           , const Eigen::VectorXd& init_value )
  {
    values_ = init_value;
    dead_band_ = dead_band;
    saturation_ = saturation;
    natural_frequency_ = natural_frequency;
    sampling_time_ = sampling_time;
    initialized_ = true;
    
    for(int i = 0; i < init_value.rows(); i++)
    {
      Eigen::VectorXd u(1); u.setZero();
      lpf.push_back(new eigen_control_toolbox::FirstOrderLowPass( natural_frequency_,sampling_time_));
      lpf.back()->setStateFromIO(u,u);
    }
  
   ROS_WARN_STREAM("Init Values......: " << values_.transpose() );
   ROS_WARN_STREAM("dead_band........: " << dead_band_.transpose() );
   ROS_WARN_STREAM("saturation.......: " << saturation_.transpose() );
   ROS_WARN_STREAM("natural_frequency: " << natural_frequency_ );
   ROS_WARN_STREAM("initialized......: " << initialized_ );
    
  }

  const Eigen::VectorXd& update( const Eigen::VectorXd& new_values )
  {
    assert( values_.rows() == new_values.rows() );
    
    Eigen::VectorXd new_values_ = new_values;
    for(int i = 0; i<new_values.size(); i++)
    {
      if(new_values[i] > dead_band_[i] )
        new_values_[i] = new_values[i] - dead_band_[i];
      else if (new_values[i] < -dead_band_[i] )
        new_values_[i] = new_values[i] + dead_band_[i];
      else 
        new_values_[i] = 0;
      
      if(new_values[i] > saturation_[i] )
        new_values_[i] = saturation_[i];
      else if (new_values[i] < -saturation_[i] )
        new_values_[i] = -saturation_[i];
      
      values_[i] = lpf[i]->update(new_values_[i]);
    }

    return values_;
  }
};


namespace iiwa_ros 
{

static const std::string FRI_CYCLE_TIME_S_NS = "fri/cycle_time_s";
static const std::string MAX_JOINT_VELOCITY_ALLOWED_NS = "fri/max_joint_velocity_allowed";

static const std::string WRENCH_FILTER_SATURATION_NS   = "fri/filter/wrench/saturation";
static const std::string WRENCH_FILTER_DEADBAND_NS     = "fri/filter/wrench/deadband";
static const std::string WRENCH_FILTER_FREQUENCY_NS    = "fri/filter/wrench/cutoff_frequency_hz";

static const std::string TWIST_FILTER_SATURATION_NS    = "fri/filter/twist/saturation";
static const std::string TWIST_FILTER_DEADBAND_NS      = "fri/filter/twist/deadband";
static const std::string TWIST_FILTER_FREQUENCY_NS     = "fri/filter/twist/cutoff_frequency_hz";

static const std::string FRI_PORT_NS                   = "fri/port";
static const std::string FRI_HOSTNAME_NS               = "fri/hostname";
static const std::string FRI_TIMEOUT_MS_NS             = "fri/timeout_ms";
static const std::string ROBOT_DESCRIPTION_NS          = "fri/robot_description_param";
static const std::string ROBOT_LINK_BASE_NS            = "fri/base_link";
static const std::string ROBOT_TOOL_BASE_NS            = "fri/tool_link";


class LBRJointOverlayClient : public KUKA::FRI::LBRClient
{


  public:
  
    LBRJointOverlayClient ( ros::NodeHandle& nh, const size_t target_queue_lenght = 1e2 );
    ~LBRJointOverlayClient( );
    
    virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
    virtual void waitForCommand();
    virtual void command();
    virtual void getState(KUKA::FRI::ESessionState& oldState, KUKA::FRI::ESessionState& newState) const;

    virtual bool newJointPosCommand( const iiwa_msgs::JointPosition& position );
    virtual bool newJointPosCommand( const std::vector< double >& new_joint_pos_command );

    virtual void newJointTorqueCommand( const iiwa_msgs::JointTorque& torque ) ;
    virtual void newJointTorqueCommand( const std::vector< double >& new_joint_torque_command );

    virtual void newWrenchCommand( const std::vector< double >& new_wrench_command ) ;
    virtual void newWrenchCommand( const iiwa_msgs::CartesianQuantity& wrench ) ;

    virtual bool getJointPosition ( iiwa_msgs::JointPosition& value ) const;
    virtual bool getJointPosition ( Eigen::Vector7d& joint_pos ) const;

    virtual bool getJointTorque ( iiwa_msgs::JointTorque& value ) const;
    virtual bool getJointTorque ( Eigen::Vector7d& joint_tau ) const;

    virtual bool getJointVelocity ( iiwa_msgs::JointVelocity& value ) const;
    virtual bool getJointVelocity ( Eigen::Vector7d& joint_vel ) const;

    virtual bool getCartesianPose ( geometry_msgs::PoseStamped& value ) const;
    virtual bool getCartesianPose ( Eigen::Affine3d& pose ) const;

    virtual bool getCartesianWrench( geometry_msgs::WrenchStamped& value, const char frame, const bool filtered = true  );
    virtual bool getCartesianWrench( Eigen::Vector6d& wrench, const char frame, const bool filtered = true ) const;

    virtual bool getCartesianTwist( Eigen::Vector6d& twist, const char frame, const bool filtered = true ) const;
    virtual bool getCartesianTwist( geometry_msgs::TwistStamped& value, const char frame, const bool filtered = true ) const;

    bool getCartesianRotation   ( Eigen::Matrix3d&    R     );
    bool getCartesianQuaternion ( Eigen::Quaterniond& quat  );
    bool getCartesianPoint      ( Eigen::Vector3d&    point );
    bool getCartesianForce      ( Eigen::Vector3d&    ret   , const char reference_frame );
    bool getCartesianTorque     ( Eigen::Vector3d&    ret   , const char reference_frame );
    bool getCartesianVelocity   ( Eigen::Vector3d&    vel   , const char reference_frame );
    bool getCartesianOmega      ( Eigen::Vector3d&    omega , const char reference_frame );


    virtual bool getJacobian(Eigen::Matrix67d& value) const;
    virtual bool getJacobian(const Eigen::Vector7d q, Eigen::Matrix67d& value) const;

    virtual bool isControlRunning( ) const;

    virtual bool updatetFirstOrderKinematic( );

    virtual Eigen::Vector7d toJointVelocity(const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega) const;
    virtual Eigen::Vector7d toJointVelocity(const Eigen::Vector7d& q, const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega) const;
    virtual Eigen::Vector6d toCartsianTwist(const Eigen::Vector7d& qd);
    virtual Eigen::Vector6d toCartsianTwist(const Eigen::VectorXd& q, const Eigen::Vector7d& qd);

    bool saturateVelocity(const Eigen::Vector7d& qd, Eigen::Vector7d& qd_saturated, double& scale ) const;
    bool saturateVelocity(const Eigen::Vector7d& qd, const Eigen::Vector7d& qd_max, Eigen::Vector7d& qd_saturated, double& scale ) const;
  private:

    ros::NodeHandle nh_;
    std::thread logger;
    void loggerThread();
    int line_;
    ros::Time last_;

    realtime_utilities::circ_buffer< std::vector<double> > command_joint_position_;
    realtime_utilities::circ_buffer< std::vector<double> > command_joint_torque_;
    realtime_utilities::circ_buffer< std::vector<double> > command_wrench_;
    std::vector<double> last_joint_pos_command_;
    std::vector<double> last_joint_torque_command_;
    std::vector<double> last_wrench_command_;
    std::vector<double> initial_joints_;
    std::vector<double> joint_pos_command_;
    std::vector<double> initial_torque_;
    std::vector<double> initial_wrench_;
    KUKA::FRI::ESessionState actual_state_;
    KUKA::FRI::ESessionState actual_state_prev_;
    bool control_running_;
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> realtime_pub_;

    KDL::Tree iiwa_tree_;
    KDL::Chain iiwa_chain_;
    std::shared_ptr< KDL::ChainFkSolverPos_recursive > fksolver_;
    std::shared_ptr< KDL::ChainJntToJacSolver        > jacsolver_;
    
    Eigen::Vector6d raw_wrench_b_;
    Eigen::Vector6d filtered_wrench_b_;
    Filter  wrench_b_filter_;

    Eigen::Vector6d   raw_twist_b_;
    Eigen::Vector6d   filtered_twist_b_;
    Filter            twist_b_filter_;

    Eigen::Affine3d   cartesian_pose_;
    Eigen::Vector7d   joint_torque_;
    Eigen::Vector7d   joint_position_;
    Eigen::Vector7d   joint_position_prev_;
    Eigen::Vector7d   joint_velocity_;
    Eigen::Vector7d   max_joint_velocity_;
    Eigen::Matrix67d  jacobian_;
    bool              jacobian_last_ok_;

    double fri_cycle_time_s_;
    ros::Time update_time_;

};

class LBROverlayApp
{
public:
  
  
  LBROverlayApp(ros::NodeHandle &nh );
    
  ~LBROverlayApp( );

  void connect();
  void disconnect();

  bool isActive() { return active_; }
  LBRJointOverlayClient& getFRIClient( ) { return client_; }
  
private:
  
  void step();
  ros::NodeHandle nh_;
  
  bool                                            active_;
  bool                                            stop_fri_command_thread_;
  std::shared_ptr<boost::thread>                  command_thread_; 
  int                                             fri_port_;
  std::string                                     fri_hostname_;
  int                                             fri_timeout_ms_;
  std::shared_ptr<KUKA::FRI::UdpConnection>       connection_;
  LBRJointOverlayClient                           client_;
  std::shared_ptr< KUKA::FRI::ClientApplication > app_;

};


}


#endif

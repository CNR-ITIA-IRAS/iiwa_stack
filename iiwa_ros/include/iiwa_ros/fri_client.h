#pragma once

#ifdef ENABLE_FRI

#include <thread>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/CartesianQuantity.h>

#include <iiwa_fri/friUdpConnection.h>
#include <iiwa_fri/friClientApplication.h>
#include <iiwa_fri/friLBRClient.h>
#include <realtime_utilities/circular_buffer.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/QR>
#include <stdio.h>

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

namespace iiwa_ros 
{
class LBRJointOverlayClient : public KUKA::FRI::LBRClient
{
private:

  std::thread logger;
  void loggerThread();
  int line_;

  public:
  
    LBRJointOverlayClient ( const std::string& robot_description
                          , const std::string& chain_root
                          , const std::string& chain_tip
                          , const size_t target_queue_lenght = 1e8 );
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

    virtual bool getJointPosition ( iiwa_msgs::JointPosition& value );
    virtual bool getJointPosition ( std::vector< double >& joint_pos );

    virtual bool getJointTorque ( iiwa_msgs::JointTorque& value );
    virtual bool getJointTorque ( std::vector< double >& joint_tau );

    virtual bool getJointVelocity ( iiwa_msgs::JointVelocity& value );
    virtual bool getJointVelocity ( std::vector< double >& joint_vel );

    virtual bool getCartesianPose ( geometry_msgs::PoseStamped& value );
    virtual bool getCartesianPose ( Eigen::Affine3d& pose );

    virtual bool getCartesianWrench( geometry_msgs::WrenchStamped& value, const char frame = 'e'  );
    virtual bool getCartesianWrench( Eigen::VectorXd& wrench, const char frame = 'e' );

    virtual bool getJacobian(Eigen::MatrixXd& value);

    virtual void getCartesianVelocity( Eigen::Affine3d& pose );

    virtual bool isControlRunning( ) const;
        
  private:
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

};

class LBROverlayApp
{
public:
  
  
  LBROverlayApp( const int& port, const std::string& hostname, int connection_timeout_ms, const std::string& robot_description, const std::string& chain_root, const std::string& chain_tip  );
    
  ~LBROverlayApp( );

  void connect();
  void disconnect();

  bool isActive() { return active_; }
  LBRJointOverlayClient& getFRIClient( ) { return client_; }
  
private:
  
  void step();
  
  bool                                            active_;
  bool                                            stop_fri_command_thread_;
  std::shared_ptr<boost::thread>                  command_thread_; 
  int                                             port_;
  std::string                                     hostname_;
  KUKA::FRI::UdpConnection                        connection_;
  LBRJointOverlayClient                           client_;
  std::shared_ptr< KUKA::FRI::ClientApplication > app_;


};


}


#endif

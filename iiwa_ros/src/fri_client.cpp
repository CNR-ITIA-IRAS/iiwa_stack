
#include <iiwa_ros/fri_client.h>

#ifdef ENABLE_FRI

#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
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


namespace iiwa_ros 
{
LBRJointOverlayClient::LBRJointOverlayClient(const size_t target_queue_lenght )
  : command_joint_position_      ( target_queue_lenght )
  , command_joint_torque_        ( target_queue_lenght )
  , command_wrench_              ( target_queue_lenght )
  , last_joint_pos_command_      ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , last_joint_torque_command_   ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , initial_joints_              ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , joint_pos_command_           ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , last_wrench_command_         ( 6, 0.0 )
  , initial_wrench_              ( 6, 0.0 )
  , control_running_             ( false )
  , update_time_                 (ros::Time::now())
{

  realtime_pub_.init( ros::NodeHandle("~"), "fri", 100);

  ros::NodeHandle nh("~");

  std::string robot_description   ;
  std::string robot_link_base     ;
  std::string robot_tool_base     ;

  if( !nh.getParam( ROBOT_DESCRIPTION_NS , robot_description ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), ROBOT_DESCRIPTION_NS .c_str()); throw std::runtime_error("Abort."); }
  if( !nh.getParam( ROBOT_LINK_BASE_NS   , robot_link_base   ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), ROBOT_LINK_BASE_NS   .c_str()); throw std::runtime_error("Abort."); }
  if( !nh.getParam( ROBOT_TOOL_BASE_NS   , robot_tool_base   ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), ROBOT_TOOL_BASE_NS   .c_str()); throw std::runtime_error("Abort."); }

  std::string robot_desc_string;
  nh.param(robot_description, robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree_))
  {
     ROS_ERROR("Failed to construct kdl tree");
     throw std::runtime_error("Failed to to construct kdl tree");
  }

  iiwa_tree_.getChain ( robot_link_base, robot_tool_base, iiwa_chain_);
  fksolver_.reset( new KDL::ChainFkSolverPos_recursive( iiwa_chain_ ) );
  jacsolver_.reset( new KDL::ChainJntToJacSolver( iiwa_chain_ ) );

  logger = std::thread( &LBRJointOverlayClient::loggerThread, this);

  std::vector<double> wrench_deadband;
  std::vector<double> wrench_saturation;
  double              wrench_freq_hz;


  std::vector<double> twist_deadband;
  std::vector<double> twist_saturation;
  double              twist_freq_hz;

  if( !nh.getParam( WRENCH_FILTER_SATURATION_NS, wrench_saturation ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), WRENCH_FILTER_SATURATION_NS.c_str()); throw std::runtime_error("Abort."); }
  if( !nh.getParam( WRENCH_FILTER_DEADBAND_NS  , wrench_deadband   ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), WRENCH_FILTER_DEADBAND_NS  .c_str()); throw std::runtime_error("Abort."); }
  if( !nh.getParam( WRENCH_FILTER_FREQUENCY_NS , wrench_freq_hz    ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), WRENCH_FILTER_FREQUENCY_NS .c_str()); throw std::runtime_error("Abort."); }

  if( !nh.getParam( TWIST_FILTER_SATURATION_NS, twist_saturation   ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), TWIST_FILTER_SATURATION_NS.c_str()) ; throw std::runtime_error("Abort."); }
  if( !nh.getParam( TWIST_FILTER_DEADBAND_NS  , twist_deadband     ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), TWIST_FILTER_DEADBAND_NS  .c_str()) ; throw std::runtime_error("Abort."); }
  if( !nh.getParam( TWIST_FILTER_FREQUENCY_NS , twist_freq_hz      ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), TWIST_FILTER_FREQUENCY_NS .c_str()) ; throw std::runtime_error("Abort."); }

  if( !nh.getParam( FRI_CYCLE_TIME_S_NS       , fri_cycle_time_s_   ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), FRI_CYCLE_TIME_S_NS .c_str()) ; throw std::runtime_error("Abort."); }

  double w_natural_frequency = wrench_freq_hz * 2 * M_PI ; // [rad/s]
  wrench_b_filter_.init( Eigen::Vector6d( wrench_deadband.data() ), Eigen::Vector6d( wrench_saturation.data() ), w_natural_frequency,  fri_cycle_time_s_, Eigen::Vector6d::Zero() );

  double t_natural_frequency = wrench_freq_hz * 2 * M_PI ; // [rad/s]
  twist_b_filter_ .init( Eigen::Vector6d( twist_deadband.data()  ), Eigen::Vector6d( twist_saturation.data()  ), t_natural_frequency ,  fri_cycle_time_s_, Eigen::Vector6d::Zero() );

  joint_position_prev_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  joint_position_     .resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  joint_torque_       .resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  joint_velocity_     .resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  jacobian_           .resize(6,KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

  jacobian_       .setZero();
  joint_velocity_ .setZero();
  joint_torque_   .setZero();

}

LBRJointOverlayClient::~LBRJointOverlayClient( ) { }

void LBRJointOverlayClient::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
{
  LBRClient::onStateChange(oldState, newState);
  switch (newState)
  {
    case KUKA::FRI::IDLE:
      break;
    case KUKA::FRI::MONITORING_WAIT:
      break;
    case KUKA::FRI::MONITORING_READY:
      initial_joints_ = std::vector<double>( robotState().getMeasuredJointPosition(), robotState().getMeasuredJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS );
      last_joint_pos_command_ = initial_joints_;
      initial_torque_ = std::vector<double>( 7, 0 );
      last_joint_torque_command_ = initial_torque_;
      initial_wrench_ = std::vector<double>( 7, 0 );
      last_wrench_command_ = initial_wrench_;
      break;

    case KUKA::FRI::COMMANDING_WAIT:
      initial_joints_ = std::vector<double>( robotState().getIpoJointPosition(), robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS );
      last_joint_pos_command_ = initial_joints_;
      initial_torque_ = std::vector<double>( 7, 0 );
      last_joint_torque_command_ = initial_torque_;
      initial_wrench_ = std::vector<double>( 7, 0 );
      last_wrench_command_ = initial_wrench_;
      break;

    case KUKA::FRI::COMMANDING_ACTIVE:
      initial_joints_ = std::vector<double>( robotState().getIpoJointPosition(), robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS );
      initial_torque_ = std::vector<double>( 7, 0 );
      control_running_ = true;
      initial_wrench_ = std::vector<double>( 7, 0 );
      break;
  }
  actual_state_prev_ = oldState;
  actual_state_ = newState;
}

void LBRJointOverlayClient::waitForCommand()
{
    // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
    // the base method.
    LBRClient::waitForCommand();

    LBRClient::command();

    // If we want to command torques, we have to command them all the time; even in
    // waitForCommand(). This has to be done due to consistency checks. In this state it is
    // only necessary, that some torque values are sent. The LBR does not take the
    // specific value into account.
    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE)
    {
        robotCommand().setTorque( &last_joint_torque_command_[0] );
    }
    else if (robotState().getClientCommandMode() == KUKA::FRI::WRENCH)
    {
        robotCommand().setWrench( &last_wrench_command_[0] );
    }
}

void LBRJointOverlayClient::command()
{
  joint_pos_command_ = last_joint_pos_command_;
  if( robotState().getClientCommandMode() == KUKA::FRI::POSITION )
  {
    if( command_joint_position_.empty() )
    {
      robotCommand().setJointPosition( &joint_pos_command_[0] );
    }
    else
    {
      joint_pos_command_ = command_joint_position_.front();
      robotCommand().setJointPosition( &joint_pos_command_[0] );
      command_joint_position_.pop_front();
    }
  }
  else if( robotState().getClientCommandMode() == KUKA::FRI::TORQUE )
  {
    LBRClient::command();
    if( command_joint_torque_.empty() )
    {
      std::vector<double> zeroTorque(7,0);
      robotCommand().setTorque( &zeroTorque[0] );
    }
    else
    {
      last_joint_torque_command_ = command_joint_torque_.front();
      robotCommand().setTorque( &last_joint_torque_command_[0] );
      command_joint_torque_.pop_front();
    }
  }
  else if ( robotState().getClientCommandMode() == KUKA::FRI::WRENCH )
  {
    LBRClient::command();
    if( command_wrench_.empty() )
    {
      std::vector<double> zeroWrench(6,0);
      robotCommand().setWrench( &last_wrench_command_[0] );
    }
    else
    {
      last_wrench_command_ = command_wrench_.front();
      robotCommand().setWrench( &last_wrench_command_[0] );
      command_wrench_.pop_front();
    }
  }

  auto const joint_pos_msr = robotState().getMeasuredJointPosition();
  auto const joint_pos_cmd_iiwa = robotState().getCommandedJointPosition();

  if (realtime_pub_.trylock())
  {
    realtime_pub_.msg_.name = {"a1_cmd", "a2_cmd", "a3_cmd", "a4_cmd","a5_cmd", "a6_cmd", "a7_cmd"
                              ,"a1_msr", "a2_msr", "a3_msr", "a4_msr","a5_msr", "a6_msr", "a7_msr"
                              ,"a1_cmd_iiwa", "a2_cmd_iiwa", "a3_cmd_iiwa", "a4_cmd_iiwa","a5_cmd_iiwa", "a6_cmd_iiwa", "a7_cmd_iiwa" };
    realtime_pub_.msg_.position = {last_joint_pos_command_[0], last_joint_pos_command_[1], last_joint_pos_command_[2]
                                 , last_joint_pos_command_[3], last_joint_pos_command_[4], last_joint_pos_command_[5], last_joint_pos_command_[6]
                                 , joint_pos_msr[0], joint_pos_msr[1], joint_pos_msr[2]
                                 , joint_pos_msr[3], joint_pos_msr[4], joint_pos_msr[5], joint_pos_msr[6]
                                 , joint_pos_cmd_iiwa[0], joint_pos_cmd_iiwa[1], joint_pos_cmd_iiwa[2]
                                 , joint_pos_cmd_iiwa[3], joint_pos_cmd_iiwa[4], joint_pos_cmd_iiwa[5], joint_pos_cmd_iiwa[6] };
    realtime_pub_.unlockAndPublish();
  }
  last_joint_pos_command_ = joint_pos_command_ ;


}


void LBRJointOverlayClient::loggerThread()
{

  ros::NodeHandle nh("~");
  ros::Publisher logger_pub = nh.advertise<std_msgs::Int16>("iiwa_line_logger", 10000);
  while( ros::ok() )
  {
    std_msgs::Int16 data_log;
    data_log.data =line_;
    logger_pub.publish( data_log );
    ros::spinOnce();
    ros::Duration(0.001).sleep();
  }

}


bool LBRJointOverlayClient::newJointPosCommand( const std::vector< double >& new_joint_pos_command )
{

  line_ = __LINE__;
  if( new_joint_pos_command.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS )
  {
    return false;
    // throw std::runtime_error("Mismatch of New Command Vector");
  }
  if( command_joint_position_.empty() )
  {
    line_ = __LINE__;
    command_joint_position_.push_back( new_joint_pos_command );
    line_ = __LINE__;
  }
  else
  {
    std::vector<double> command_joint_position = command_joint_position_.back();
    line_ = __LINE__;
    command_joint_position_.push_back( new_joint_pos_command );
    line_ = __LINE__;
  }
  return true;
}

bool LBRJointOverlayClient::newJointPosCommand( const iiwa_msgs::JointPosition& position )
{
  line_ = __LINE__;
  std::vector<double> new_joint_pos_command( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 );
  line_ = __LINE__;
  new_joint_pos_command[0] = position.position.a1; line_ = __LINE__;
  new_joint_pos_command[1] = position.position.a2; line_ = __LINE__;
  new_joint_pos_command[2] = position.position.a3; line_ = __LINE__;
  new_joint_pos_command[3] = position.position.a4; line_ = __LINE__;
  new_joint_pos_command[4] = position.position.a5; line_ = __LINE__;
  new_joint_pos_command[5] = position.position.a6; line_ = __LINE__;
  new_joint_pos_command[6] = position.position.a7; line_ = __LINE__;
  
  return newJointPosCommand( new_joint_pos_command );
}

void LBRJointOverlayClient::newJointTorqueCommand( const iiwa_msgs::JointTorque& torque )
{
  std::vector<double> new_joint_torque_command( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 );
  new_joint_torque_command[0] = torque.torque.a1;
  new_joint_torque_command[1] = torque.torque.a2;
  new_joint_torque_command[2] = torque.torque.a3;
  new_joint_torque_command[3] = torque.torque.a4;
  new_joint_torque_command[4] = torque.torque.a5;
  new_joint_torque_command[5] = torque.torque.a6;
  new_joint_torque_command[6] = torque.torque.a7;
  return newJointTorqueCommand( new_joint_torque_command );
}

void LBRJointOverlayClient::newJointTorqueCommand( const std::vector< double >& new_joint_torque_command )
{
  if( robotState().getClientCommandMode() != KUKA::FRI::TORQUE )
  {
    ROS_WARN_STREAM(ros::Time::now() << " fri_client.h - wrong command state in newJointTorqueCommand ");
    return;
  }

  if( new_joint_torque_command.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS )
  {
    throw std::runtime_error("Mismatch of New Command Vector");
  }
  command_joint_torque_.push_back( new_joint_torque_command );

  return;
}

void LBRJointOverlayClient::newWrenchCommand( const iiwa_msgs::CartesianQuantity& wrench )
{
  std::vector<double> new_wrench_command( 6, 0.0 );
  new_wrench_command[0] = wrench.x;
  new_wrench_command[1] = wrench.y;
  new_wrench_command[2] = wrench.z;
  new_wrench_command[3] = wrench.a;
  new_wrench_command[4] = wrench.b;
  new_wrench_command[5] = wrench.c;
  return newWrenchCommand( new_wrench_command );
}

void LBRJointOverlayClient::newWrenchCommand( const std::vector< double >& new_wrench_command )
{
  while( robotState().getSessionState() != KUKA::FRI::COMMANDING_ACTIVE )
  {
    ros::Duration(0.005).sleep();
  }

  if( robotState().getClientCommandMode() != KUKA::FRI::WRENCH )
  {
    ROS_WARN_STREAM(ros::Time::now() << " fri_client.h - wrong command state in newWrenchCommand ");
    return;
  }

  if( new_wrench_command.size() != 6 )
  {
    throw std::runtime_error("Mismatch of New Command Vector");
  }
  command_wrench_.push_back( new_wrench_command );

  return;
}

void LBRJointOverlayClient::getState(KUKA::FRI::ESessionState& oldState, KUKA::FRI::ESessionState& newState) const
{
  oldState = actual_state_prev_;
  newState = actual_state_;
}

bool LBRJointOverlayClient::getJointPosition(Eigen::VectorXd &joint_pos ) const
{
    joint_pos = joint_position_;
    return true;
}

bool LBRJointOverlayClient::getJointPosition ( iiwa_msgs::JointPosition& value ) const
{
  value.position.a1 = joint_position_( 0 ) ;
  value.position.a2 = joint_position_( 1 ) ;
  value.position.a3 = joint_position_( 2 ) ;
  value.position.a4 = joint_position_( 3 ) ;
  value.position.a5 = joint_position_( 4 ) ;
  value.position.a6 = joint_position_( 5 ) ;
  value.position.a7 = joint_position_( 6 ) ;
  value.header.stamp = update_time_;
  return true;
}

bool LBRJointOverlayClient::getJointTorque( Eigen::VectorXd& joint_tau ) const
{
    joint_tau = joint_torque_;
    return true;
}

bool LBRJointOverlayClient::getJointTorque ( iiwa_msgs::JointTorque& value ) const
{
  value.torque.a1 = joint_torque_( 0 ) ;
  value.torque.a2 = joint_torque_( 1 ) ;
  value.torque.a3 = joint_torque_( 2 ) ;
  value.torque.a4 = joint_torque_( 3 ) ;
  value.torque.a5 = joint_torque_( 4 ) ;
  value.torque.a6 = joint_torque_( 5 ) ;
  value.torque.a7 = joint_torque_( 6 ) ;
  value.header.stamp = update_time_;
  return true;
}

bool LBRJointOverlayClient::getJointVelocity(Eigen::VectorXd &joint_vel ) const
{
  joint_vel = joint_velocity_;
  return true;
}

bool LBRJointOverlayClient::getJointVelocity ( iiwa_msgs::JointVelocity& value ) const
{
  value.velocity.a1 = joint_velocity_( 0 ) ;
  value.velocity.a2 = joint_velocity_( 1 ) ;
  value.velocity.a3 = joint_velocity_( 2 ) ;
  value.velocity.a4 = joint_velocity_( 3 ) ;
  value.velocity.a5 = joint_velocity_( 4 ) ;
  value.velocity.a6 = joint_velocity_( 5 ) ;
  value.velocity.a7 = joint_velocity_( 6 ) ;
  value.header.stamp = update_time_;
  return true;
}

bool LBRJointOverlayClient::getCartesianPose( Eigen::Affine3d& pose ) const
{
  pose = cartesian_pose_;
  return true;
}

bool LBRJointOverlayClient::getCartesianPose ( geometry_msgs::PoseStamped& value ) const
{
  tf::poseEigenToMsg(cartesian_pose_, value.pose);
  value.header.stamp = update_time_;
  return true;
}

bool LBRJointOverlayClient::getCartesianWrench( Eigen::VectorXd& wrench, const char frame, const bool filtered ) const
{

    Eigen::VectorXd wrench_b = filtered ? filtered_wrench_b_ : raw_wrench_b_;
    if( frame == 'b')
    {
      wrench = wrench_b;
    }
    else
    {
      wrench.resize(6);
      wrench.setZero();
      Eigen::Vector3d f; f << wrench_b(0),wrench_b(1),wrench_b(2);
      Eigen::Vector3d t; t << wrench_b(3),wrench_b(4),wrench_b(5);
      f = cartesian_pose_.linear().transpose() * f;
      t = cartesian_pose_.linear().transpose() * t;
      wrench << f(0),f(1),f(2),t(0),t(1),t(2);
    }
    return true;
}

bool LBRJointOverlayClient::getCartesianWrench (geometry_msgs::WrenchStamped& value, const char frame  , const bool filtered)
{
  Eigen::VectorXd wrench(7); wrench.setZero();
  if(! getCartesianWrench(wrench, frame, filtered )  )
    return false;

  value.wrench.force.x  = wrench(0);
  value.wrench.force.y  = wrench(1);
  value.wrench.force.z  = wrench(2);
  value.wrench.torque.x = wrench(3);
  value.wrench.torque.y = wrench(4);
  value.wrench.torque.z = wrench(5);

  value.header.stamp = update_time_;
  return true;
}

bool LBRJointOverlayClient::getJacobian(Eigen::MatrixXd& value) const
 {
   value = jacobian_;
   return true;
 }

bool LBRJointOverlayClient::getCartesianTwist(Eigen::Vector6d &twist , const char frame, const bool filtered) const
{
  Eigen::VectorXd twist_b = filtered ? filtered_twist_b_ : raw_twist_b_;
  if( frame == 'b')
  {
    twist = twist_b;
  }
  else
  {
    twist.resize(6);
    twist.setZero();
    Eigen::Vector3d v; v << twist_b(0),twist_b(1),twist_b(2);
    Eigen::Vector3d o; o << twist_b(3),twist_b(4),twist_b(5);
    v = cartesian_pose_.linear().transpose() * v;
    o = cartesian_pose_.linear().transpose() * o;
    twist << v(0),v(1),v(2),o(0),o(1),o(2);
  }
  return true;
}

bool LBRJointOverlayClient::getCartesianTwist( geometry_msgs::TwistStamped& value, const char frame, const bool filtered) const
{
  Eigen::Vector6d twist; twist.setZero();
  if(! getCartesianTwist(twist, frame, filtered )  )
    return false;

  value.twist.linear.x  = twist(0);
  value.twist.linear.y  = twist(1);
  value.twist.linear.z  = twist(2);
  value.twist.angular.x = twist(3);
  value.twist.angular.y = twist(4);
  value.twist.angular.z = twist(5);

  value.header.stamp = update_time_;
  return true;
}


bool LBRJointOverlayClient::updatetFirstOrderKinematic( )
{
  static bool first_entry = true;

  const KUKA::FRI::LBRState& state = LBRClient::robotState();
  const double*                jp  = state.getMeasuredJointPosition();

  update_time_ = ros::Time::now();

  //////// VELOCITY
  Eigen::VectorXd joint_Velocity_prev; joint_Velocity_prev.resize(7);
  for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
  {
    joint_position_(i) = jp[i];
  }

  if( first_entry )
  {
    for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
    {
      joint_position_prev_(i) = jp[i];
    }
    first_entry = false;
  }
  joint_velocity_ = ( joint_position_- joint_position_prev_ ) / fri_cycle_time_s_;
//  ROS_INFO_STREAM_THROTTLE(5,"---\njp     : " << joint_position_.transpose()
//                             << "\njp_prev: " << joint_position_prev_.transpose()
//                             << "\ndjp    : " << (joint_position_-joint_position_prev_).transpose()
//                             << "\ndt     : " << fri_cycle_time_s_
//                             << "\njv     : " << joint_velocity_.transpose() );
  joint_position_prev_ = joint_position_;
  //////// VELOCITY

  //////// TORQUE
  const double* tau = state.getExternalTorque();
  for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
    joint_torque_(i) = tau[i];
  //////// TORQUE

  //////// POSE
  KDL::Frame      fr;
  unsigned int    nj = iiwa_tree_.getNrOfJoints();
  KDL::JntArray   ja = KDL::JntArray(nj);
  for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
    ja(i) = jp[i];

  bool kinematics_status = fksolver_->JntToCart(ja,fr);
  tf::transformKDLToEigen(fr, cartesian_pose_ );
  //////// POSE

  //////// JACOBIAN
  KDL::Jacobian jac;
  jac.resize(iiwa_chain_.getNrOfJoints());
  jacsolver_->JntToJac (ja, jac );

  assert( jac.columns() == 7 );
  assert( jac.rows() == 6 );
  for(size_t i=0; i<6; i++)
    for(size_t j=0; j<7; j++)
      jacobian_(i,j) = jac(i,j);
  //////// JACOBIAN

  Eigen::MatrixXd pinv = jacobian_.completeOrthogonalDecomposition().pseudoInverse();

  raw_wrench_b_ = pinv.transpose() * joint_torque_;
  filtered_wrench_b_ = wrench_b_filter_.update( raw_wrench_b_ );

  raw_twist_b_ = jacobian_ * joint_velocity_;
  filtered_twist_b_ = twist_b_filter_.update( raw_twist_b_ );

  return true;
}



bool LBRJointOverlayClient::isControlRunning( ) const { return control_running_; }

/**
 * @brief LBROverlayApp::LBROverlayApp
 * @param port
 * @param hostname
 * @param connection_timeout_ms
 * @param robot_description
 * @param chain_root
 * @param chain_tip
 */
LBROverlayApp::LBROverlayApp()
: active_    ( false )
{

  ros::NodeHandle nh("~");
  if( !nh.getParam( FRI_PORT_NS          , fri_port_        ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), FRI_PORT_NS          .c_str()); throw std::runtime_error("Abort."); }
  if( !nh.getParam( FRI_HOSTNAME_NS      , fri_hostname_    ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), FRI_HOSTNAME_NS      .c_str()); throw std::runtime_error("Abort."); }
  if( !nh.getParam( FRI_TIMEOUT_MS_NS    , fri_timeout_ms_  ) ) { ROS_ERROR("Param %s/%s not in param server. Abort. ", nh.getNamespace().c_str(), FRI_TIMEOUT_MS_NS    .c_str()); throw std::runtime_error("Abort."); }

  connection_.reset( new KUKA::FRI::UdpConnection( (unsigned int)fri_timeout_ms_ ) );
  connect();
}

/**
 * @brief LBROverlayApp::~LBROverlayApp
 */
LBROverlayApp::~LBROverlayApp( )
{
  ROS_INFO("Destroy the JointOverlayApp (disconnect the FRI)");
  disconnect();
}

/**
 * @brief LBROverlayApp::connect
 */
void LBROverlayApp::connect()
{
  ROS_WARN("FRI Connect to %s:%d ", fri_hostname_.c_str(), fri_port_ );
  app_.reset(new KUKA::FRI::ClientApplication(*connection_, client_) );
  if( !app_->connect(fri_port_, fri_hostname_.size() > 0 ? fri_hostname_.c_str() : nullptr ) )
  {
    throw std::runtime_error( ("Error in FRI Connection (" + fri_hostname_ + ":" + std::to_string( fri_port_ ) ).c_str() );
  }
  stop_fri_command_thread_ = false;
  command_thread_.reset( new boost::thread(boost::bind(&LBROverlayApp::step, this)) );
  ROS_WARN("FRI Connect. Done.");
}
/**
 * @brief LBROverlayApp::disconnect
 */
void LBROverlayApp::disconnect()
{
  if( app_ )
  {
    ROS_WARN("FRI Disconnect");
    app_->disconnect();

    stop_fri_command_thread_ = true;
    command_thread_->join();
    command_thread_.reset();

    app_.reset();
    ROS_WARN("FRI Cleanup. Done.");

  }
}

/**
 * @brief LBROverlayApp::step
 */
void LBROverlayApp::step()
{
  bool success = true;
  while( (!stop_fri_command_thread_) && ros::ok() && success )
  {
    if( app_ )
    {
      KUKA::FRI::EConnectionQuality quality = client_.robotState().getConnectionQuality();
      switch( quality )
      {
        case KUKA::FRI::POOR      : ROS_FATAL_THROTTLE(5, "** POOR ** FRI COMMUNICATION STATE     " ); break;
        case KUKA::FRI::FAIR      : ROS_WARN_THROTTLE (5, "** FAIR ** FRI COMMUNICATION STATE     " ); break;
        case KUKA::FRI::GOOD      : ROS_WARN_THROTTLE (5, "** GOOD ** FRI COMMUNICATION STATE     " ); break;
        case KUKA::FRI::EXCELLENT : ROS_INFO_THROTTLE (5, "** EXCELLENT** FRI COMMUNICATION STATE " ); break;
      }

      KUKA::FRI::ESessionState oldState, newState;
      client_.getState(oldState,newState);
      if( client_.isControlRunning() && ((int)oldState > (int)KUKA::FRI::IDLE) && ( newState == KUKA::FRI::IDLE ) )
      {
       ROS_WARN("Shutdown transition detected.(%d/%d)", (int) oldState, (int)newState );
      }
      active_ = (newState == KUKA::FRI::COMMANDING_ACTIVE);
      if( active_ )
      {
        if(! client_.updatetFirstOrderKinematic( ) )
        {
          ROS_FATAL_THROTTLE(2,"Error in update the kinematics and static update....");
        }
      }
      success = app_->step();
    }
    else
    {
      ROS_INFO("App reset. Exit. ");
      break;
    }
  }
}

}


#endif

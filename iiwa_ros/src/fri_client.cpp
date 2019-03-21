
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
LBRJointOverlayClient::LBRJointOverlayClient( const std::string& robot_description
                                            , const std::string& chain_root
                                            , const std::string& chain_tip
                                            , const size_t target_queue_lenght )
  : command_joint_position_         ( target_queue_lenght )
  , command_joint_torque_           ( target_queue_lenght )
  , command_wrench_                 ( target_queue_lenght )
  , last_joint_pos_command_         ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , last_joint_torque_command_      ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , initial_joints_                 ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , joint_pos_command_              ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
  , last_wrench_command_            ( 6, 0.0 )
  , initial_wrench_                 ( 6, 0.0 )
  , control_running_                ( false )
{

  realtime_pub_.init( ros::NodeHandle("~"), "fri", 100);

  ros::NodeHandle nh("~");
  std::string robot_desc_string;
  nh.param(robot_description, robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree_))
  {
     ROS_ERROR("Failed to construct kdl tree");
     throw std::runtime_error("Failed to to construct kdl tree");
  }

  iiwa_tree_.getChain ( chain_root, chain_tip, iiwa_chain_);
  fksolver_.reset( new KDL::ChainFkSolverPos_recursive( iiwa_chain_ ) );
  jacsolver_.reset( new KDL::ChainJntToJacSolver( iiwa_chain_ ) );

  // logger = std::thread( &LBRJointOverlayClient::loggerThread, this);
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
      //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
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
      //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
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
    line_ = __LINE__;
    std::vector<double> command_joint_position = command_joint_position_.back();
    line_ = __LINE__;
    if( diff( command_joint_position, new_joint_pos_command ) > 0.001 * M_PI / 180.0 )
    {
      line_ = __LINE__;
      command_joint_position_.push_back( new_joint_pos_command );
      line_ = __LINE__;
    }
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
  while( robotState().getSessionState() != KUKA::FRI::COMMANDING_ACTIVE )
  {
    ros::Duration(0.005).sleep();
  }

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

//       std::cout<< "new commandded wrench: " << new_wrench_command[0] << "; " << new_wrench_command[1] << "; " << new_wrench_command[2] << "; " << std::endl;

  return;
}

void LBRJointOverlayClient::getState(KUKA::FRI::ESessionState& oldState, KUKA::FRI::ESessionState& newState) const
{
  oldState = actual_state_prev_;
  newState = actual_state_;
}

bool LBRJointOverlayClient::getJointPosition( std::vector< double >& joint_pos )
{
    const KUKA::FRI::LBRState& state = LBRClient::robotState();
    const double* jp = state.getMeasuredJointPosition();
    joint_pos.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,0);
    for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
      joint_pos.at(i) = jp[i];

    return true;
}

bool LBRJointOverlayClient::getJointPosition ( iiwa_msgs::JointPosition& value )
{
  std::vector<double> jp(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  if(! getJointPosition(jp) )
    return false;
  value.position.a1 = jp[0] ;
  value.position.a2 = jp[1] ;
  value.position.a3 = jp[2] ;
  value.position.a4 = jp[3] ;
  value.position.a5 = jp[4] ;
  value.position.a6 = jp[5] ;
  value.position.a7 = jp[6] ;

  value.header.stamp = ros::Time::now();
  return true;
}

bool LBRJointOverlayClient::getJointTorque( std::vector< double >& joint_tau )
{
    const KUKA::FRI::LBRState& state = LBRClient::robotState();
    const double* tau = state.getMeasuredTorque();
    joint_tau.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,0);
    for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
      joint_tau.at(i) = tau[i];

    return true;
}

bool LBRJointOverlayClient::getJointTorque ( iiwa_msgs::JointTorque& value )
{
  std::vector<double> tau(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  if(! getJointTorque(tau) )
    return false;
  value.torque.a1 = tau[0] ;
  value.torque.a2 = tau[1] ;
  value.torque.a3 = tau[2] ;
  value.torque.a4 = tau[3] ;
  value.torque.a5 = tau[4] ;
  value.torque.a6 = tau[5] ;
  value.torque.a7 = tau[6] ;
  value.header.stamp = ros::Time::now();
  return true;
}

bool LBRJointOverlayClient::getJointVelocity( std::vector< double >& joint_vel )
{
  static int first_entry = 1;
  static timespec act_time_prev;
  static double jp_prev[KUKA::FRI::LBRState::NUMBER_OF_JOINTS] = {0};

  joint_vel.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,0.0);

  const KUKA::FRI::LBRState& state = LBRClient::robotState();
  const double*              jp   =  state.getMeasuredJointPosition();
  if( first_entry )
  {
      first_entry = 0;
      clock_gettime(CLOCK_MONOTONIC, &act_time_prev);
      std::memcpy(&jp_prev, jp, sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
      return true;
  }

  timespec act_time;
  clock_gettime(CLOCK_MONOTONIC, &act_time);
  const double dt = timer_difference_s(&act_time, &act_time_prev);
  joint_vel.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS,0);
  for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
      joint_vel.at(i) = ( jp[i] - jp_prev[i] ) / dt;

  std::memcpy(&jp_prev, jp, sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  act_time_prev.tv_sec  = act_time.tv_sec;
  act_time_prev.tv_nsec = act_time.tv_nsec;
  return true;
}

bool LBRJointOverlayClient::getJointVelocity ( iiwa_msgs::JointVelocity& value )
{
  std::vector<double> jv(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  if(! getJointVelocity(jv) )
    return false;
  value.velocity.a1 = jv[0] ;
  value.velocity.a2 = jv[1] ;
  value.velocity.a3 = jv[2] ;
  value.velocity.a4 = jv[3] ;
  value.velocity.a5 = jv[4] ;
  value.velocity.a6 = jv[5] ;
  value.velocity.a7 = jv[6] ;

  value.header.stamp = ros::Time::now();
  return true;
}

bool LBRJointOverlayClient::getCartesianPose( Eigen::Affine3d& pose )
{
    const KUKA::FRI::LBRState& state = LBRClient::robotState();
    const double* jp = state.getMeasuredJointPosition();

    KDL::Frame      fr;
    unsigned int    nj = iiwa_tree_.getNrOfJoints();
    KDL::JntArray   ja = KDL::JntArray(nj);
    for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
      ja(i) = jp[i];

    bool kinematics_status = fksolver_->JntToCart(ja,fr);
    tf::transformKDLToEigen(fr, pose );

    return true;
}

bool LBRJointOverlayClient::getCartesianPose ( geometry_msgs::PoseStamped& value )
{
  Eigen::Affine3d pose;
  if(! getCartesianPose(pose) )
    return false;
  tf::poseEigenToMsg(pose, value.pose);
  value.header.stamp = ros::Time::now();
  return true;
}

bool LBRJointOverlayClient::getCartesianWrench( Eigen::VectorXd& wrench, const char frame )
{
    const KUKA::FRI::LBRState& state = LBRClient::robotState();
    const double* jp  = state.getMeasuredJointPosition();
    const double* tau = state.getExternalTorque();
    Eigen::VectorXd torque(7);
    for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
      torque(i) = tau[i];

    //-------------------
    Eigen::Affine3d pose;
    KDL::Frame      fr;
    unsigned int    nj = iiwa_tree_.getNrOfJoints();
    KDL::JntArray   ja = KDL::JntArray(nj);
    for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
      ja(i) = jp[i];

    bool kinematics_status = fksolver_->JntToCart(ja,fr);
    tf::transformKDLToEigen(fr, pose );


    //-------------------
    KDL::Jacobian jac;
    jac.resize(iiwa_chain_.getNrOfJoints());
    jacsolver_->JntToJac (ja, jac );
    Eigen::MatrixXd jacobian(6,7); jacobian.setZero();
    assert( jac.columns() == 7 );
    assert( jac.rows() == 6 );
    for(size_t i=0; i<6; i++)
      for(size_t j=0; j<7; j++)
        jacobian(i,j) = jac(i,j);
    //-------------------

    Eigen::MatrixXd pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::VectorXd wrench_b = pinv.transpose() * torque;
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
      f = pose.linear().transpose() * f;
      t = pose.linear().transpose() * t;
      wrench << f(0),f(1),f(2),t(0),t(1),t(2);
    }

    return true;
}

bool LBRJointOverlayClient::getCartesianWrench ( geometry_msgs::WrenchStamped& value, const char frame  )
{
  Eigen::VectorXd wrench(7); wrench.setZero();
  if(! getCartesianWrench(wrench, frame)  )
    return false;

  value.wrench.force.x  = wrench(0);
  value.wrench.force.y  = wrench(1);
  value.wrench.force.z  = wrench(2);
  value.wrench.torque.x = wrench(3);
  value.wrench.torque.y = wrench(4);
  value.wrench.torque.z = wrench(5);

  value.header.stamp = ros::Time::now();
  return true;
}

bool LBRJointOverlayClient::getJacobian(Eigen::MatrixXd& value)
 {
   const KUKA::FRI::LBRState& state = LBRClient::robotState();
   const double* jp  = state.getMeasuredJointPosition();

   KDL::Frame      fr;
   unsigned int    nj = iiwa_tree_.getNrOfJoints();
   KDL::JntArray   ja = KDL::JntArray(nj);
   for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
     ja(i) = jp[i];

   KDL::Jacobian jac;
   jac.resize(iiwa_chain_.getNrOfJoints());
   jacsolver_->JntToJac (ja, jac );
   value.resize(6,7);
   value.setZero();
   for(size_t i=0; i<6; i++)
     for(size_t j=0; j<7; j++)
       value(i,j) = jac(i,j);

   return true;
 }

void LBRJointOverlayClient::getCartesianVelocity( Eigen::Affine3d& pose )
{
    const KUKA::FRI::LBRState& state = LBRClient::robotState();
    const double* jp = state.getMeasuredJointPosition();

    KDL::Frame      fr;
    unsigned int    nj = iiwa_tree_.getNrOfJoints();
    KDL::JntArray   ja = KDL::JntArray(nj);
    for(size_t i=0;i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS;i++)
      ja(i) = jp[i];

    bool kinematics_status = fksolver_->JntToCart(ja,fr);
    tf::transformKDLToEigen(fr, pose );
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
LBROverlayApp::LBROverlayApp( const int& port, const std::string& hostname, int connection_timeout_ms, const std::string& robot_description, const std::string& chain_root, const std::string& chain_tip  )
    : port_      ( port )
    , hostname_  ( hostname )
    , connection_( connection_timeout_ms )
    , client_    ( robot_description, chain_root, chain_tip )
    , active_    ( false )
  {
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
  ROS_WARN("FRI Connect to %s:%d ", hostname_.c_str(), port_ );
  app_.reset(new KUKA::FRI::ClientApplication(connection_, client_) );
  if( !app_->connect(port_, hostname_.size() > 0 ? hostname_.c_str() : nullptr ) )
  {
    throw std::runtime_error( ("Error in FRI Connection (" + hostname_ + ":" + std::to_string( port_ ) ).c_str() );
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
         case KUKA::FRI::EXCELLENT : ROS_INFO_THROTTLE (5, "** EXCELLENT** FRI COMMUNICATION STATE     " ); break;
      }
      KUKA::FRI::ESessionState oldState, newState;
      client_.getState(oldState,newState);
      if( client_.isControlRunning() && ((int)oldState > (int)KUKA::FRI::IDLE) && ( newState == KUKA::FRI::IDLE ) )
      {
       ROS_WARN("Shutdown transition detected.(%d/%d)", (int) oldState, (int)newState );
      }
      active_ = (newState == KUKA::FRI::COMMANDING_ACTIVE);

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

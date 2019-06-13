/**
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef ENABLE_FRI
// #include <iiwa_msgs/ControlMode.h>
#endif
#include <eigen3/Eigen/Core>
#include <iiwa_msgs/ControlMode.h>
#include <iiwa_ros/iiwa_ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using namespace std;

namespace iiwa_ros
{

  
ros::Time last_update_time;

iiwaRos::iiwaRos() { }

void iiwaRos::init (ros::NodeHandle &nh, double fri_cycle_time, const bool verbosity)
{
  servo_motion_service_.reset(new iiwa_ros::ServoMotion(nh));
  
  dt_ = fri_cycle_time;
  holder_state_pose_.init ( "state/CartesianPose" );
  holder_state_joint_position_.init ( "state/JointPosition" );
  holder_state_joint_torque_.init ( "state/JointTorque" );
  holder_state_wrench_.init ( "state/CartesianWrench" );
  holder_state_joint_stiffness_.init ( "state/JointStiffness" );
  holder_state_joint_position_velocity_.init ( "state/JointPositionVelocity" );
  holder_state_joint_damping_.init ( "state/JointDamping" );
  holder_state_joint_velocity_.init ( "state/JointVelocity" );
  holder_state_destination_reached_.init ( "state/DestinationReached" );

  holder_command_pose_.init ( "command/CartesianPose" );
  holder_command_payload_.init ( "command/Payload" );
  holder_command_joint_position_.init ( "command/JointPosition" );
  holder_command_joint_position_velocity_.init ( "command/JointPositionVelocity" );
  holder_command_joint_velocity_.init ( "command/JointVelocity" );

  servo_motion_service_->setServiceName ( "configuration/configureSmartServo" );
  path_parameters_service_.setServiceName ( "configuration/pathParameters" );
  time_to_destination_service_.setServiceName ( "state/timeToDestination" );
  
  servo_motion_service_->setVerbosity(verbosity);
  path_parameters_service_.setVerbosity(verbosity);
  time_to_destination_service_.setVerbosity(verbosity);

  stop_fri_publisher_thread_ = false;

}

void iiwaRos::friPublisherThread()
#define CATCH_ERROR(X) if(!X){ROS_ERROR_THROTTLE(10,"%s failed", #X);}
{
  ros::NodeHandle nh("~");

  ros::Publisher state_joint_position_pub  = nh.advertise<iiwa_msgs::JointPosition>      ( "/iiwa/state/JointPosition"                             , 10  );
  ros::Publisher state_joint_torque_pub    = nh.advertise<iiwa_msgs::JointTorque>        ( "/iiwa/state/JointTorque"                               , 10  );
  ros::Publisher state_joint_velocity_pub  = nh.advertise<iiwa_msgs::JointVelocity>      ( "/iiwa/state/JointVelocity"                             , 10  );

  ros::Publisher state_pose_pub            = nh.advertise<geometry_msgs::PoseStamped>    ( "/iiwa/state/CartesianPose"                             , 10  );

  ros::Publisher state_fwrench_b_pub       = nh.advertise<geometry_msgs::WrenchStamped>  ( "/iiwa/state/base/filtered/CartesianWrench"             , 10 );
  ros::Publisher state_rwrench_b_pub       = nh.advertise<geometry_msgs::WrenchStamped>  ( "/iiwa/state/base/raw/CartesianWrench"                  , 10 );
  ros::Publisher state_cwrench_b_pub       = nh.advertise<geometry_msgs::WrenchStamped>  ( "/iiwa/state/base/payload_compensated/CartesianWrench"  , 10 );

  ros::Publisher state_fwrench_t_pub       = nh.advertise<geometry_msgs::WrenchStamped>  ( "/iiwa/state/tool/filtered/CartesianWrench"             , 10 );
  ros::Publisher state_rwrench_t_pub       = nh.advertise<geometry_msgs::WrenchStamped>  ( "/iiwa/state/tool/raw/CartesianWrench"                  , 10 );
  ros::Publisher state_cwrench_t_pub       = nh.advertise<geometry_msgs::WrenchStamped>  ( "/iiwa/state/tool/payload_compensated/CartesianWrench"  , 10 );

  ros::Publisher state_ftwist_b_pub        = nh.advertise<geometry_msgs::TwistStamped>   ( "/iiwa/state/base/filtered/CartesianTwist"              , 10 );
  ros::Publisher state_rtwist_b_pub        = nh.advertise<geometry_msgs::TwistStamped>   ( "/iiwa/state/base/raw/CartesianTwist"                   , 10 );

  ros::Publisher state_ftwist_t_pub        = nh.advertise<geometry_msgs::TwistStamped>   ( "/iiwa/state/tool/filtered/CartesianTwist"              , 10 );
  ros::Publisher state_rtwist_t_pub        = nh.advertise<geometry_msgs::TwistStamped>   ( "/iiwa/state/tool/raw/CartesianTwist"                   , 10 );


  double default_publisher_freq = 50.;
  double publisher_freq = 100.;
  if(!nh.getParam(FRI_STATE_PUBISHER_FREQ_HS_NS, publisher_freq) )
  {
    ROS_WARN("The iiwa state update publisher frequency is not set. The default value equal to %fHz is superimposed", publisher_freq);
    publisher_freq = default_publisher_freq;
  }

  ros::Rate rt( publisher_freq );
  while( ros::ok && ! stop_fri_publisher_thread_ )
  {


    iiwa_msgs::JointPosition      jp        ; CATCH_ERROR( getJointPosition      (jp       )                        );
    iiwa_msgs::JointTorque        jt        ; CATCH_ERROR( getJointTorque        (jt       )                        );
    iiwa_msgs::JointVelocity      jv        ; CATCH_ERROR( getJointVelocity      (jv       )                        );

    geometry_msgs::PoseStamped    pose      ; CATCH_ERROR( getCartesianPose      (pose     )                        );

    geometry_msgs::WrenchStamped  fwrench_b ; CATCH_ERROR( getCartesianWrench    (fwrench_b , 'b', true , false)    );
    geometry_msgs::WrenchStamped  rwrench_b ; CATCH_ERROR( getCartesianWrench    (rwrench_b , 'b', false, false)    );
    geometry_msgs::WrenchStamped  cwrench_b ; CATCH_ERROR( getCartesianWrench    (cwrench_b , 'b', true , true )    );

    geometry_msgs::WrenchStamped  fwrench_t ; CATCH_ERROR( getCartesianWrench    (fwrench_t , 't', true , false)    );
    geometry_msgs::WrenchStamped  rwrench_t ; CATCH_ERROR( getCartesianWrench    (rwrench_t , 't', false, false)    );
    geometry_msgs::WrenchStamped  cwrench_t ; CATCH_ERROR( getCartesianWrench    (cwrench_t , 't', true , true )    );

    geometry_msgs::TwistStamped   ftwist_b  ; CATCH_ERROR( getCartesianTwist     (ftwist_b  , 'b', true )           );
    geometry_msgs::TwistStamped   rtwist_b  ; CATCH_ERROR( getCartesianTwist     (rtwist_b  , 'b', false)           );

    geometry_msgs::TwistStamped   ftwist_t  ; CATCH_ERROR( getCartesianTwist     (ftwist_t  , 't', true )           );
    geometry_msgs::TwistStamped   rtwist_t  ; CATCH_ERROR( getCartesianTwist     (rtwist_t  , 't', false)           );

    state_joint_position_pub.publish( jp        );
    state_joint_torque_pub  .publish( jt        );
    state_joint_velocity_pub.publish( jv        );

    state_pose_pub          .publish( pose      );

    state_fwrench_b_pub     .publish( fwrench_b );
    state_rwrench_b_pub     .publish( rwrench_b );
    state_cwrench_b_pub     .publish( cwrench_b );

    state_fwrench_t_pub     .publish( fwrench_t );
    state_rwrench_t_pub     .publish( rwrench_t );
    state_cwrench_t_pub     .publish( cwrench_t );

    state_ftwist_b_pub      .publish( ftwist_b  );
    state_rtwist_b_pub      .publish( rtwist_b  );

    state_ftwist_t_pub      .publish( ftwist_t  );
    state_rtwist_t_pub      .publish( rtwist_t  );

    rt.sleep();
  }
#undef CATCH_ERROR
}

void iiwaRos::startFriPublisher()
{
  stop_fri_publisher_thread_ = false;
  while( !isFRIModalityActive() && ros::ok() )
  {
      ROS_WARN_THROTTLE( 2, "The FRI is not yet started... " );
  }
  fri_publisher_thread_.reset( new std::thread( &iiwaRos::friPublisherThread, this ) );
}

void iiwaRos::stopFriPublisher()
{
  stop_fri_publisher_thread_ = true;
  fri_publisher_thread_->join();
  fri_publisher_thread_.reset(  );
}

bool iiwaRos::getRobotIsConnected()
{
  if( !isFRIModalityActive() )
  {
    ros::Duration diff = ( ros::Time::now() - last_update_time );
    return ( diff < ros::Duration ( 0.25 ) );
  }
  else
  {
    return true;
  }
}

bool iiwaRos::isFRIModalityActive()
{
  int control_modality_active = servo_motion_service_->getControlModeActive();
  bool fri_modality_active    = ( control_modality_active == iiwa_msgs::ControlMode::FRI_JOINT_POS_CONTROL  )
                              || ( control_modality_active == iiwa_msgs::ControlMode::FRI_TORQUE_CONTROL    )
                              || ( control_modality_active == iiwa_msgs::ControlMode::FRI_JOINT_IMP_CONTROL );

  return fri_modality_active 
      ||( servo_motion_service_->getFRIApp( ) != nullptr && servo_motion_service_->getFRIApp( )->isActive() );

}

bool iiwaRos::getJointPosition ( iiwa_msgs::JointPosition& value )
{
  if( !isFRIModalityActive() )
  {
    return holder_state_joint_position_.get ( value );
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getJointPosition(value);
  }
}

bool iiwaRos::getJointPosition( Eigen::Vector7d& value )
{
  if( !isFRIModalityActive() )
  {
    iiwa_msgs::JointPosition jp;
    if(!holder_state_joint_position_.get ( jp ) )
      return false;
    iiwa_ros::iiwaJointPositionToEigenVector(jp,value);
    return true;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getJointPosition(value);
  }
}

bool iiwaRos::getJointTorque ( iiwa_msgs::JointTorque& value )
{
  if( !isFRIModalityActive() )
    return holder_state_joint_torque_.get ( value );
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getJointTorque(value);
  }
}

bool iiwaRos::getJointTorque ( Eigen::Vector7d& value )
{
  if( !isFRIModalityActive() )
  {
    iiwa_msgs::JointTorque jt;
    if(!holder_state_joint_torque_.get ( jt ) )
      return false;
    iiwa_ros::iiwaJointTorqueToEigenVector(jt,value);
    return true;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getJointTorque(value);
  }
}

bool iiwaRos::getJointVelocity ( iiwa_msgs::JointVelocity& value )
{
  if( !isFRIModalityActive() )
    return holder_state_joint_velocity_.get ( value );
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getJointVelocity (value);
  }
}

bool iiwaRos::getJointVelocity ( Eigen::Vector7d& value )
{
  if( !isFRIModalityActive() )
  {
    iiwa_msgs::JointVelocity jv;
    if(!holder_state_joint_velocity_.get ( jv ) )
      return false;
    iiwa_ros::iiwaJointVelocityToEigenVector(jv,value);
    return true;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getJointVelocity (value);
  }
}

bool iiwaRos::getCartesianPose( geometry_msgs::PoseStamped& value ) {
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianPose(value);
  }
  return true;
}

bool iiwaRos::getCartesianPose( Eigen::Affine3d& value ) {
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianPose(value);
  }
  return true;
}

bool iiwaRos::getCartesianWrench (geometry_msgs::WrenchStamped& value, const char what, const bool filtered, const bool compensate_payload ) {
  bool ret = true;

  Eigen::Vector6d  wrench = Eigen::Vector6d::Zero();
  if( !isFRIModalityActive() )
  {
    ret = holder_state_wrench_.get ( value );
  }
  else
  {
    ret = servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianWrench (value, what, filtered);
  }

  if(!ret)
    return false;

  wrench(0) = value.wrench.force.x;
  wrench(1) = value.wrench.force.y;
  wrench(2) = value.wrench.force.z;
  wrench(3) = value.wrench.torque.x;
  wrench(4) = value.wrench.torque.y;
  wrench(5) = value.wrench.torque.z;

  Eigen::Vector6d wrench_ret = Eigen::Vector6d::Zero();
  if (compensate_payload && payload.initializated)
  {

    geometry_msgs::PoseStamped pose_msg;
    while(! getCartesianPose(pose_msg) )
    {
      ROS_ERROR("Waiting for a good cart position");
      ros::Duration(0.05).sleep();
    }

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    if(payload.compensation_method == Payload::PAYLOAD_ESTIMATION )
    {

      Eigen::Vector3d gravity_b; gravity_b << 0,0,-9.81;
      Eigen::Vector3d gravity_e = pose.linear().transpose() * gravity_b;

      if(what == 'b')
      { // "wrench" is in base frame
        wrench_ret.block(0,0,3,1) = wrench.block(0,0,3,1) - payload.mass * gravity_b;
        wrench_ret.block(3,0,3,1) = wrench.block(3,0,3,1) - pose.linear() * (payload.mass * payload.distance.cross(gravity_e));
      }
      else
      { // "wrench" is in tool frame
        wrench_ret.block(0,0,3,1) = wrench.block(0,0,3,1) - payload.mass * gravity_e;
        wrench_ret.block(3,0,3,1) = wrench.block(3,0,3,1) - payload.mass * payload.distance.cross(gravity_e);
      }
    }
    else
    {
      if(what == 'b')
      {
        wrench_ret = wrench - payload.wrench_offset_b;
      }
      else
      { // "wrench" is in tool frame
        wrench_ret.block(0,0,3,1) = pose.linear().transpose( ) * (pose.linear() * wrench.block(0,0,3,1) - payload.wrench_offset_b.block(0,0,3,1) );
        wrench_ret.block(3,0,3,1) = pose.linear().transpose( ) * (pose.linear() * wrench.block(3,0,3,1) - payload.wrench_offset_b.block(3,0,3,1) );
      }
    }
  }
  else
  {
      wrench_ret = wrench;
  }

  value.header.stamp = ros::Time::now();
  tf::wrenchEigenToMsg(wrench_ret,value.wrench);
  return true;
}

bool iiwaRos::getCartesianWrench (Eigen::Vector6d& value, const char what, const bool filtered, const bool compensate_payload ) {
  geometry_msgs::WrenchStamped wrench;
  if(!getCartesianWrench (wrench, what, filtered, compensate_payload ) )
  {
    return false;
  }
  tf::wrenchMsgToEigen(wrench.wrench,value);
  return true;
}

bool iiwaRos::getCartesianTwist( geometry_msgs::TwistStamped& value, const char what, const bool filtered  )
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianTwist(value, what, filtered);
  }
  return true;
}

bool iiwaRos::getCartesianTwist( Eigen::Vector6d& value, const char what, const bool filtered  )
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianTwist(value, what, filtered);
  }
  return true;
}


bool iiwaRos::getCartesianRotation   ( Eigen::Matrix3d&    R     )
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianRotation(R);
  }
  return true;
}
bool iiwaRos::getCartesianQuaternion ( Eigen::Quaterniond& quat  )
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianQuaternion(quat);
  }
  return true;
}

bool iiwaRos::getCartesianPoint      ( Eigen::Vector3d&    point )
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getCartesianPoint(point);
  }
  return true;
}
bool iiwaRos::getCartesianForce ( Eigen::Vector3d&    ret, const char reference_frame, const bool filtered, const bool compensate_payload)
{
  Eigen::Vector6d wrench;
  if( !getCartesianWrench(wrench, reference_frame, filtered, compensate_payload))
    return false;
   ret =  wrench.block(0,0,3,1);
   return true;
}
bool iiwaRos::getCartesianTorque ( Eigen::Vector3d&    ret   , const char reference_frame, const bool filtered, const bool compensate_payload  )
{
  Eigen::Vector6d wrench;
  if( !getCartesianWrench(wrench, reference_frame, filtered, compensate_payload))
    return false;
   ret =  wrench.block(3,0,3,1);
   return true;
}

bool iiwaRos::getCartesianVelocity ( Eigen::Vector3d& vel, const char reference_frame, const bool filtered )
{
  Eigen::Vector6d twist;
  if( !getCartesianTwist(twist, reference_frame, filtered))
    return false;
  vel =  twist.block(3,0,3,1);
 return true;
}

bool iiwaRos::getCartesianOmega ( Eigen::Vector3d& omega , const char reference_frame, const bool filtered )
{
  Eigen::Vector6d twist;
  if( !getCartesianTwist(twist, reference_frame, filtered))
    return false;
  omega =  twist.block(3,0,3,1);
 return true;
}

Eigen::Vector7d iiwaRos::toJointVelocity(const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega)
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    assert(0);
    return Eigen::Vector7d::Zero();
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().toJointVelocity(velocity, omega);
  }
}
Eigen::Vector7d iiwaRos::toJointVelocity(const Eigen::VectorXd& q, const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega)
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    assert(0);
    return Eigen::Vector7d::Zero();
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().toJointVelocity(q, velocity, omega);
  }
}
bool iiwaRos::saturateVelocity(const Eigen::Vector7d& qd, Eigen::Vector7d& qd_saturated, double& scale )
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    assert(0);
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().saturateVelocity(qd, qd_saturated, scale );
  }
}

bool iiwaRos::saturateVelocity(const Eigen::Vector7d& qd, const Eigen::Vector7d& qd_max, Eigen::Vector7d& qd_saturated, double& scale )
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    assert(0);
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().saturateVelocity(qd, qd_max, qd_saturated, scale );
  }
}



Eigen::Vector6d iiwaRos::toCartsianTwist(const Eigen::Vector7d& qd)
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    assert(0);
    return Eigen::Vector6d::Zero();
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().toCartsianTwist(qd);
  }
}
Eigen::Vector6d iiwaRos::toCartsianTwist(const Eigen::VectorXd& q, const Eigen::Vector7d& qd)
{
  if( !isFRIModalityActive() )
  {
    ROS_ERROR_THROTTLE(5,"Not yet supported");
    assert(0);
    return Eigen::Vector6d::Zero();
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().toCartsianTwist(q, qd);
  }
}

bool iiwaRos::getJacobian(Eigen::Matrix67d& value)
{
  if( !isFRIModalityActive() )
  {
    assert(0);
    return false;
  }
  else
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().getJacobian (value);
  }
}

bool iiwaRos::getJointStiffness ( iiwa_msgs::JointStiffness& value )
{
    return holder_state_joint_stiffness_.get ( value );
}

bool iiwaRos::getJointDamping ( iiwa_msgs::JointDamping& value )
{
    return holder_state_joint_damping_.get ( value );
}

void iiwaRos::setCartesianPose ( const geometry_msgs::PoseStamped& position )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_->getControlModeActive();
  if(( control_modality_active == 5 )
  || ( control_modality_active == 6 )
  || ( control_modality_active == 7 )
  || ( control_modality_active == 8 ) 
  )
  {
    throw std::runtime_error( "FRI : TODO" );
  }
  else
  {
    holder_command_pose_.set ( position );
    holder_command_pose_.publishIfNew();
  }
#else
    holder_command_pose_.set ( position );
    holder_command_pose_.publishIfNew();
#endif
}

void iiwaRos::setPayload ( const geometry_msgs::PoseStamped& position )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_->getControlModeActive();
  if(( control_modality_active == 5 )
  || ( control_modality_active == 6 )
  || ( control_modality_active == 7 )
  || ( control_modality_active == 8 ) 
  )
  {
    ROS_INFO( "setting the new payload..." );
    ROS_INFO_STREAM( " position.pose ");
    holder_command_payload_.set ( position );
    holder_command_payload_.publishIfNew();
    ros::Duration(3.).sleep();
  }
  else
  {
    ROS_INFO( "setting the new payload..." );
    holder_command_payload_.set ( position );
    holder_command_payload_.publishIfNew();
    ros::Duration(3.).sleep();
  }
#else
    holder_command_payload_.set ( position );
    holder_command_payload_.publishIfNew();
#endif
}

void iiwaRos::setJointTorque ( const iiwa_msgs::JointTorque& torque )
{
#if defined( ENABLE_FRI )
  if( ( servo_motion_service_->getControlModeActive() == 6 ) )
  {
    servo_motion_service_->getFRIApp( )->getFRIClient().newJointTorqueCommand( torque );
  }
  else
  {
//     holder_command_joint_position_.set ( position );
//     holder_command_joint_position_.publishIfNew();
  }
#else
//   holder_command_joint_position_.set ( position );
//   holder_command_joint_position_.publishIfNew();
#endif
}

void iiwaRos::setWrench ( const iiwa_msgs::CartesianQuantity& wrench )
{
#if defined( ENABLE_FRI )
  if( ( servo_motion_service_->getControlModeActive() == 8 ) )
  {
    servo_motion_service_->getFRIApp( )->getFRIClient().newWrenchCommand( wrench );
  }
  else
  {
//     holder_command_joint_position_.set ( position );
//     holder_command_joint_position_.publishIfNew();
  }
#else
//   holder_command_joint_position_.set ( position );
//   holder_command_joint_position_.publishIfNew();
#endif
}

bool iiwaRos::setJointPosition ( const iiwa_msgs::JointPosition& position )
{

#if defined( ENABLE_FRI )
  if( isFRIModalityActive() )
  {
    return servo_motion_service_->getFRIApp( )->getFRIClient().newJointPosCommand( position );
  }
  else
  {
    holder_command_joint_position_.set ( position );
    holder_command_joint_position_.publishIfNew();
    return true;
  }
#else
  holder_command_joint_position_.set ( position );
  holder_command_joint_position_.publishIfNew();
#endif

}

void iiwaRos::setJointVelocity ( const iiwa_msgs::JointVelocity& velocity )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_->getControlModeActive();
  if( ( control_modality_active == 5 )
  ||  ( control_modality_active == 6 ) 
  ||  ( control_modality_active == 7 ) )
  {
    throw std::runtime_error( "FRI : TODO" );
  }
  else
  {
    holder_command_joint_velocity_.set ( velocity );
    holder_command_joint_velocity_.publishIfNew();
  }
#else
  holder_command_joint_velocity_.set ( velocity );
  holder_command_joint_velocity_.publishIfNew();
#endif

}

bool iiwaRos::estimatePayload(const double estimation_time, const double toll)
{
  ROS_INFO("Estimation of the payload (no movement foreseen");
  ros::Time st = ros::Time::now();
  
  geometry_msgs::WrenchStamped wrench_msg;
  geometry_msgs::PoseStamped   pose_msg;
  Eigen::Vector6d              wrench = Eigen::Vector6d::Zero();
  Eigen::Affine3d              pose;
  Eigen::Vector3d              torque; torque.setZero();
  
  payload.compensation_method = Payload::PAYLOAD_ESTIMATION;
  
  ROS_INFO("Get Pose");
  if(!getCartesianPose(pose_msg))
  {
    ROS_ERROR("Problem in getCartesianPose");
    return false;
  }
  
  tf::poseMsgToEigen(pose_msg.pose,pose);
  
  int count = 0;
  ros::Rate r(50);
  
  ROS_INFO("Get some data");
  while((ros::Time::now() - st).toSec() < estimation_time)
  {
    count ++;
    if(!getCartesianWrench(wrench_msg,'e', true, false))
    {
      ROS_ERROR("Problem in getCartesianWrench");
      return false;
    }
    
     Eigen::Matrix<double,6,1> wrench_tmp;
    tf::wrenchMsgToEigen(wrench_msg.wrench,wrench_tmp);
    
    wrench += wrench_tmp;
    r.sleep();
  }
  
  ROS_INFO("Estimate");
  
  wrench /= (double)count;
  payload.mass = wrench.block(0,0,3,1).norm()/9.81;
  
  Eigen::Vector3d gravity_b; gravity_b <<0, 0, -9.81;
  Eigen::Vector3d gravity_e = pose.linear().transpose() * gravity_b;
  
  torque = wrench.block(3,0,3,1);
  
  //Tx_e = m(-gz*dy + gy*dz)   --> dz = ( Tx_e/m + gz*dy)/gy
  //Ty_e = m(gz*dx - gx*dz )   --> dz = (-Ty_e/m + gz*dx)/gx 
  //Tz_e = m(-gy*dx + gx*dy)
  if(torque(2)/payload.mass > ( gravity_e(0)*toll + gravity_e(1)*toll))
    ROS_WARN("Warning: torque along z direction not negligible");
  
  if(gravity_e(0) < 0.01 && gravity_e(1) > 0.01)
    payload.distance <<0, 0,  (torque(0)/payload.mass)/gravity_e(1);
  else if(gravity_e(0) > 0.01 && gravity_e(1) < 0.01)
    payload.distance <<0, 0, -torque(1)/payload.mass/gravity_e(0);
  else if(gravity_e(0) < 0.01 && gravity_e(1) < 0.01)
  {
    ROS_ERROR("Impossible to compute the distance. return");
    return false;
  }
  else 
    payload.distance <<0, 0, 0.5 * ( (torque(0)/payload.mass)/gravity_e(1) + -torque(1)/payload.mass/gravity_e(0) );
  
  ROS_INFO("Estimation finished");
  
  payload.initializated = true;
  return true;
}

bool iiwaRos::setWrenchOffset(const double estimation_time)
{
  ROS_INFO("Estimation of the payload (no movement foreseen");
  ros::Time st = ros::Time::now();
  
  geometry_msgs::WrenchStamped wrench_msg;
  geometry_msgs::PoseStamped   pose_msg;
  Eigen::Vector6d              wrench = Eigen::Vector6d::Zero();

  
  payload.compensation_method = Payload::OFFSET_ESTIMATION;
  
  int count = 0;
  ros::Rate r(50);
  
  while((ros::Time::now() - st).toSec() < estimation_time)
  {
    count++;
    if(!getCartesianWrench(wrench_msg,'b', false, false ))
    {
      ROS_ERROR("Problem in getCartesianWrench");
      return false;
    }
    
    Eigen::Matrix<double,6,1> wrench_tmp;
    tf::wrenchMsgToEigen(wrench_msg.wrench,wrench_tmp);
    
    wrench += wrench_tmp;
    r.sleep();
  }
  wrench /= (double) count;
  payload.wrench_offset_b = wrench;
  
  
  ROS_INFO_STREAM("Estimation finished: "  << payload.wrench_offset_b.transpose() );
  geometry_msgs::WrenchStamped value;
  getCartesianWrench (value, 'b', true, true );
  ROS_INFO_STREAM( " Wrench in Base, filtered and compensated: " << value.wrench );
  getCartesianWrench (value, 'b', false, true );
  ROS_INFO_STREAM( " Wrench in Base, raw and compensated: " << value.wrench );
  getCartesianWrench (value, 'b', false, false );
  ROS_INFO_STREAM( " Wrench in Base, raw: " << value.wrench );
  payload.initializated = true;
  return true;
}
   
void iiwaRos::setJointPositionVelocity ( const iiwa_msgs::JointPositionVelocity& value )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_->getControlModeActive();
  if( ( control_modality_active == 5 )
  ||  ( control_modality_active == 6 ) 
  ||  ( control_modality_active == 7 ) )
  {
    throw std::runtime_error( "FRI : TODO" );
  }
  else
  {
    holder_command_joint_position_velocity_.set ( value );
    holder_command_joint_position_velocity_.publishIfNew();
  }
#else
    holder_command_joint_position_velocity_.set ( value );
    holder_command_joint_position_velocity_.publishIfNew();
#endif
}

}

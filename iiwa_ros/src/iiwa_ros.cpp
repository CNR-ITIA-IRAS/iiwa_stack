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

#include <iiwa_ros/iiwa_ros.h>

using namespace std;

namespace iiwa_ros
{

ros::Time last_update_time;

iiwaRos::iiwaRos() { }

void iiwaRos::init(const bool verbosity)
{
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

    servo_motion_service_.setServiceName ( "configuration/configureSmartServo" );
    path_parameters_service_.setServiceName ( "configuration/pathParameters" );
    time_to_destination_service_.setServiceName ( "state/timeToDestination" );
    
    servo_motion_service_.setVerbosity(verbosity);
    path_parameters_service_.setVerbosity(verbosity);
    time_to_destination_service_.setVerbosity(verbosity);
}

bool iiwaRos::getRobotIsConnected()
{
    ros::Duration diff = ( ros::Time::now() - last_update_time );
    return ( diff < ros::Duration ( 0.25 ) );
}

bool iiwaRos::getCartesianPose ( geometry_msgs::PoseStamped& value )
{
    return holder_state_pose_.get ( value );
}

bool iiwaRos::getJointPosition ( iiwa_msgs::JointPosition& value )
{
    return holder_state_joint_position_.get ( value );
}

bool iiwaRos::getJointTorque ( iiwa_msgs::JointTorque& value )
{
    return holder_state_joint_torque_.get ( value );
}

bool iiwaRos::getJointStiffness ( iiwa_msgs::JointStiffness& value )
{
    return holder_state_joint_stiffness_.get ( value );
}

bool iiwaRos::getCartesianWrench ( geometry_msgs::WrenchStamped& value )
{
    return holder_state_wrench_.get ( value );
}

bool iiwaRos::getJointVelocity ( iiwa_msgs::JointVelocity& value )
{
    return holder_state_joint_velocity_.get ( value );
}

bool iiwaRos::getJointPositionVelocity ( iiwa_msgs::JointPositionVelocity& value )
{
    return holder_state_joint_position_velocity_.get ( value );
}

bool iiwaRos::getJointDamping ( iiwa_msgs::JointDamping& value )
{
    return holder_state_joint_damping_.get ( value );
}

void iiwaRos::setCartesianPose ( const geometry_msgs::PoseStamped& position )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_.getControlModeActive();
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
  int control_modality_active = servo_motion_service_.getControlModeActive();
  if(( control_modality_active == 5 )
  || ( control_modality_active == 6 )
  || ( control_modality_active == 7 )
  || ( control_modality_active == 8 ) 
  )
  {
    std::cout << "setting the new payload..." << std::endl;
    std::cout << position.pose << std::endl;
    holder_command_payload_.set ( position );
    holder_command_payload_.publishIfNew();
    ros::Duration(3.).sleep();
  }
  else
  {
    std::cout << "setting the new payload..." << std::endl;
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
  if( ( servo_motion_service_.getControlModeActive() == 6 ) )
  {
    servo_motion_service_.getFriClient()->newJointTorqueCommand( torque );
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
  if( ( servo_motion_service_.getControlModeActive() == 8 ) )
  {
    servo_motion_service_.getFriClient()->newWrenchCommand( wrench );
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

void iiwaRos::setJointPosition ( const iiwa_msgs::JointPosition& position, double goal_tolerance )
{
#if defined( ENABLE_FRI )

  int control_modality_active = servo_motion_service_.getControlModeActive();
  if( ( control_modality_active == 5 )
  ||  ( control_modality_active == 7 ) )
  {
    servo_motion_service_.getFriClient()->newJointPosCommand( position );
  }
  else
  {
    holder_command_joint_position_.set ( position );
    holder_command_joint_position_.publishIfNew();
  }
#else
  holder_command_joint_position_.set ( position );
  holder_command_joint_position_.publishIfNew();
#endif
  iiwa_msgs::JointPosition j_cmd = position;
  iiwa_msgs::JointPosition j_msr;
  while( 1 )
  {
    getJointPosition ( j_msr );
    double dist = std::sqrt( std::pow( j_cmd.position.a1-j_msr.position.a1, 2 )
                           + std::pow( j_cmd.position.a2-j_msr.position.a2, 2 )
                           + std::pow( j_cmd.position.a3-j_msr.position.a3, 2 )
                           + std::pow( j_cmd.position.a4-j_msr.position.a4, 2 )
                           + std::pow( j_cmd.position.a5-j_msr.position.a5, 2 )
                           + std::pow( j_cmd.position.a6-j_msr.position.a6, 2 )
                           + std::pow( j_cmd.position.a7-j_msr.position.a7, 2 ) );
    if( ( dist < 0.05 ) || ( dist < goal_tolerance ) ) 
    {
      break;
    }
    ros::Duration(0.005).sleep();
  }
}

void iiwaRos::setJointVelocity ( const iiwa_msgs::JointVelocity& velocity )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_.getControlModeActive();
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
void iiwaRos::setJointPositionVelocity ( const iiwa_msgs::JointPositionVelocity& value )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_.getControlModeActive();
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

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

#pragma once

#ifdef ENABLE_FRI
  #include <iiwa_fri/friUdpConnection.h>
  #include <iiwa_fri/friClientApplication.h>
  #include <iiwa_fri/friLBRClient.h>
  #include <realtime_utilities/circular_buffer.h>
#endif

#include <ros/ros.h>
#include <iiwa_ros/common.h>
#include <iiwa_ros/servo_motion_service.h>
#include <iiwa_ros/path_parameters_service.h>
#include <iiwa_ros/time_to_destination_service.h>



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
#include <iostream>
#include <thread>
#include <string>

namespace iiwa_ros 
{
  
  extern ros::Time last_update_time;
  
  template <typename ROSMSG>
  class iiwaHolder {
  public:
    iiwaHolder() : is_new(false) {}
    
    void set_value(const ROSMSG& value) {
      mutex.lock();
      data = value;
      is_new = true;
      mutex.unlock();
    }
    
    bool get_value(ROSMSG& value) {
      bool was_new = false;
      
      mutex.lock();
      value = data;
      was_new = is_new;
      is_new = false;
      mutex.unlock();
      
      return was_new;
    }
    
    bool has_new_value() {
      return is_new;
    }
    
    ROSMSG get_value_unsynchronized() {
      return data;
    }
    
  private:
    ROSMSG data;
    bool is_new;
    std::mutex mutex;
  };
  
  template <typename ROSMSG>
  class iiwaStateHolder {
  public:
    void init(const std::string& topic) {
      ros::NodeHandle nh;
      subscriber = nh.subscribe<ROSMSG>(topic, 1, &iiwaStateHolder<ROSMSG>::set, this);
    }
    
    bool has_new_value() {
      return holder.has_new_value();
    }
    
    void set(ROSMSG value) {
      last_update_time = ros::Time::now();
      holder.set_value(value);
    }
    
    bool get(ROSMSG& value) {
      return holder.get_value(value);        
    }
  private:
    iiwaHolder<ROSMSG> holder;
    ros::Subscriber subscriber;
  };
  
  
  template <typename ROSMSG>
  class iiwaCommandHolder {
  public:
    void init(const std::string& topic) {
      ros::NodeHandle nh;
      publisher = nh.advertise<ROSMSG>(topic, 1);
    }
    
    void set(const ROSMSG& value) {
      holder.set_value(value);
    }
    
    ROSMSG get() {
      return holder.get_value_unsynchronized();
    }
    
    void publishIfNew() {
      static ROSMSG msg;
      if (publisher.getNumSubscribers() && holder.get_value(msg))
        publisher.publish(msg);
    }
  private:
    ros::Publisher publisher;
    iiwaHolder<ROSMSG> holder;
  };
  

  static const std::string FRI_STATE_PUBISHER_FREQ_HS_NS = "fir/state_publisher_freq_hz";
  
  class iiwaRos 
  {
  public:
    
    /**
     * @brief Constructor for class iiwaRos holding all the methods to command and get the state of the robot.
     */
    iiwaRos();
    
    /**
     * @brief Initializes the necessary topics for state and command methods.
     * 
     * @return void
     */
    void init ( ros::NodeHandle &nh
              , double fri_cycle_time
              , const bool verbosity = false );
    
    bool estimatePayload(const double estimation_time = 5, const double toll = 0.005);
    bool setWrenchOffset(const double estimation_time = 5 );
    
    double getFRICycleTime() const { return dt_;}
    void  startFriPublisher() ;
    void  stopFriPublisher();
    
    /**
     * @brief Returns true is a new Cartesian pose of the robot is available.
     * 
     * @param value the current Cartesian Pose of the robot.
     * @return bool
     */
    bool getCartesianPose(geometry_msgs::PoseStamped& value);
    bool getCartesianPose(Eigen::Affine3d& pose );
    
    /**
     * @brief Returns true is a new Joint position of the robot is available.
     * 
     * @param value the current joint position of the robot.
     * @return bool
     */
    bool getJointPosition(iiwa_msgs::JointPosition& value);
    bool getJointPosition(Eigen::Vector7d& value);
    
    /**
     * @brief Returns true is a new Joint torque of the robot is available.
     * 
     * @param value the current joint torque of the robot.
     * @return bool
     */
    bool getJointTorque(iiwa_msgs::JointTorque& value);
    bool getJointTorque(Eigen::Vector7d& value);


    /**
     * @brief Returns true is a new Joint position velocity of the robot is available.
     *
     * @param value the current joint position velocity of the robot.
     * @return bool
     */
    bool getJointVelocity(iiwa_msgs::JointVelocity& value);
    bool getJointVelocity(Eigen::Vector7d& value);
    /**
     * @brief Returns true is a new Cartesian wrench of the robot is available.
     * 
     * @param value the current cartesian wrench of the robot.
     * @return bool
     */
    bool getCartesianWrench(geometry_msgs::WrenchStamped& value, const char reference_frame = 'e', const bool filtered = true, const bool compensate_payload = true );
    bool getCartesianWrench(Eigen::Vector6d& value, const char reference_frame = 'e', const bool filtered = true, const bool compensate_payload = true );

    bool getCartesianTwist( geometry_msgs::TwistStamped& twist, const char reference_frame = 'e', const bool filtered = true  );
    bool getCartesianTwist( Eigen::Vector6d& twist, const char reference_frame = 'e', const bool filtered = true  );

    Eigen::Vector7d toJointVelocity(const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega);
    Eigen::Vector7d toJointVelocity(const Eigen::VectorXd& q, const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega);
    Eigen::Vector6d toCartsianTwist(const Eigen::Vector7d& qd);
    Eigen::Vector6d toCartsianTwist(const Eigen::VectorXd& q, const Eigen::Vector7d& qd);

    bool saturateVelocity(const Eigen::Vector7d& qd, Eigen::Vector7d& qd_saturated, double& scale );
    bool saturateVelocity(const Eigen::Vector7d& qd, const Eigen::Vector7d& qd_max, Eigen::Vector7d& qd_saturated, double& scale );

    bool getCartesianRotation   ( Eigen::Matrix3d&    R     );
    bool getCartesianQuaternion ( Eigen::Quaterniond& quat  );
    bool getCartesianPoint      ( Eigen::Vector3d&    point );
    bool getCartesianForce      ( Eigen::Vector3d&    ret   , const char reference_frame, const bool filtered = true, const bool compensate_payload = true );
    bool getCartesianTorque     ( Eigen::Vector3d&    ret   , const char reference_frame, const bool filtered = true, const bool compensate_payload = true );
    bool getCartesianVelocity   ( Eigen::Vector3d&    vel   , const char reference_frame, const bool filtered = true );
    bool getCartesianOmega      ( Eigen::Vector3d&    omega , const char reference_frame, const bool filtered = true );

    /**
     * @brief Returns true is a new Joint stiffness of the robot is available.
     *
     * @param value the current joint stiffness of the robot.
     * @return bool
     */
    bool getJointStiffness(iiwa_msgs::JointStiffness& value);
    /**
     * @brief Returns true is a new Joint damping of the robot is available.
     * 
     * @param value the current joint damping of the robot.
     * @return bool
     */
    bool getJointDamping(iiwa_msgs::JointDamping& value);

    bool getJacobian(Eigen::Matrix67d &value);
    
    /**
     * @brief Returns the object that allows to call the configureSmartServo service.
     * 
     * @return iiwa_ros::ServoMotion
     */
    ServoMotion& getServoMotion() { return * servo_motion_service_; }
    
    /**
     * @brief Returns the object that allows to call the timeToDestination service.
     * 
     * @return iiwa_ros::PathParametersService
     */
#if BACK_COMPATIBILITY_IIWA_STACK
    PathParametersService getPathParametersService() { return path_parameters_service_; }
    
    /**
     * @brief Returns the object that allows to call the setPathParameters service.
     * 
     * @return iiwa_ros::TimeToDestinationService
     */
    TimeToDestinationService getTimeToDestinationService() { return time_to_destination_service_; };
#endif
    
    bool payloadInitialized() const {return payload.initializated; };
    
    /**
     * @brief Set the cartesian pose of the robot.
     * 
     * @param position the cartesian pose to set the robot.
     * @return void
     */
    void setCartesianPose(const geometry_msgs::PoseStamped& position);
    
    void setPayload(const geometry_msgs::PoseStamped& payload); // payload.pose.position.x = mass; payload.pose.position.z = distance from flange; 
    
    /**
     * @brief Set the joint position of the robot.
     * 
     * @param position the joint position to set the robot.
     * @return void
     */
    bool setJointPosition(const iiwa_msgs::JointPosition& position );
    
    void setJointTorque(const iiwa_msgs::JointTorque& torque );
    
    void setWrench(const iiwa_msgs::CartesianQuantity& wrench );
    
    /**
     * @brief Set the joint velocity of the robot.
     * 
     * @param velocity the joint velocity to set the robot.
     * @return void
     */
    void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);
    
    /**
     * @brief Set the joint position velocity of the robot.
     * 
     * @param value the joint position velocity of the robot.
     * @return void
     */
    void setJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value);
    
    /**
     * \brief Returns the current connection status of an IIWA robot.
     */
    bool getRobotIsConnected();
    bool isFRIModalityActive();
    
  private:
    #if BACK_COMPATIBILITY_IIWA_STACK
    iiwaStateHolder<geometry_msgs::PoseStamped> holder_state_pose_;
    iiwaStateHolder<iiwa_msgs::JointPosition> holder_state_joint_position_;
    iiwaStateHolder<iiwa_msgs::JointTorque> holder_state_joint_torque_;
    iiwaStateHolder<geometry_msgs::WrenchStamped> holder_state_wrench_;
    iiwaStateHolder<iiwa_msgs::JointDamping> holder_state_joint_damping_;
    iiwaStateHolder<iiwa_msgs::JointStiffness> holder_state_joint_stiffness_;
    iiwaStateHolder<iiwa_msgs::JointVelocity> holder_state_joint_velocity_;
    iiwaStateHolder<iiwa_msgs::JointPositionVelocity> holder_state_joint_position_velocity_;
    iiwaStateHolder<std_msgs::Time> holder_state_destination_reached_;
    
    iiwaCommandHolder<geometry_msgs::PoseStamped> holder_command_pose_;
    iiwaCommandHolder<geometry_msgs::PoseStamped> holder_command_payload_;
    iiwaCommandHolder<iiwa_msgs::JointPosition> holder_command_joint_position_;
    iiwaCommandHolder<iiwa_msgs::JointVelocity> holder_command_joint_velocity_;
    iiwaCommandHolder<iiwa_msgs::JointPositionVelocity> holder_command_joint_position_velocity_;
    
    PathParametersService path_parameters_service_;
    TimeToDestinationService time_to_destination_service_;
#endif
    
    std::shared_ptr< ServoMotion > servo_motion_service_;
        
    struct Payload
    {
      enum CompensationMethod { PAYLOAD_ESTIMATION, OFFSET_ESTIMATION };
      CompensationMethod compensation_method;
      double mass;
      Eigen::Vector3d distance;
      bool initializated;
      Eigen::Vector6d wrench_offset_b;
      Payload() : compensation_method(Payload::PAYLOAD_ESTIMATION), mass(0), distance(Eigen::Vector3d::Zero()), initializated(false),wrench_offset_b(Eigen::Vector6d::Zero() ){}
    } payload;

    double dt_;

    bool stop_fri_publisher_thread_;
    void friPublisherThread();
    std::shared_ptr<std::thread> fri_publisher_thread_;

};
  

}

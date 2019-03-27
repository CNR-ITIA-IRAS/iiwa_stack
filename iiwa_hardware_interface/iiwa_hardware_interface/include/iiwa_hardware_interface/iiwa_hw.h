/** 
 * This class implements a bridge between ROS hardware interfaces and a KUKA LBR IIWA Robot,
 * using an IIWARos communication described in the iiwa_ros package.
 *
 * It is a porting of the work from the Centro E. Piaggio in Pisa : https://github.com/CentroEPiaggio/kuka-lwr
 * for the LBR IIWA. We acknowledge the good work of their main contributors :
 * Carlos J. Rosales - cjrosales@gmail.com
 * Enrico Corvaglia
 * Marco Esposito - marco.esposito@tum.de
 * Manuel Bonilla - josemanuelbonilla@gmail.com
 * 
 * LICENSE :
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

#ifndef __ITIA_TOPIC_HARDWARE_INTERFACE__
#define __ITIA_TOPIC_HARDWARE_INTERFACE__

#include <mutex>

#include <urdf/model.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <itia_basic_hardware_interface/itia_basic_hardware_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <itia_basic_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <iiwa_msgs/ControlMode.h>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointPositionVelocity.h>

#include <iiwa_ros/iiwa_ros.h>

namespace itia_hardware_interface
{

///////////////////////////////////////////////////////////////////////////////////////////////
constexpr int DEFAULT_CONTROL_FREQUENCY = 1000; // Hz
constexpr int IIWA_JOINTS = 7;


//< The IIWA supports different interpolator (Servo), called "Motion"
enum IIWA_MOTION_TYPE : int { IIWA_SMART_MOTION                  = 0
                            , IIWA_DIRECT_MOTION                 = 1
                            , IIWA_SMART_MOTION_LIN              = 2 };

//< The setpoint (command) can be expressed following different covention according to the interpolator (Motion)
enum IIWA_COMMAND_TYPE : int  { IIWA_CMD_NONE                      = -1
                              , IIWA_CMD_CARTESIAN_POSE            =  0
                              , IIWA_CMD_CARTESIAN_POSE_LIN        =  1
                              , IIWA_CMD_CARTESIAN_VELOCITY        =  2
                              , IIWA_CMD_JOINT_POSITION            =  3
                              , IIWA_CMD_JOINT_POSITION_VELOCITY   =  4
                              , IIWA_CMD_JOINT_VELOCITY            =  5 };
    
const static std::map< IIWA_COMMAND_TYPE, std::string > IIWA_COMMAND_TYPE_IDS
    { { IIWA_CMD_NONE                      , "CMD_NONE"                      } 
    , { IIWA_CMD_CARTESIAN_POSE            , "CMD_CARTESIAN_POSE"            } 
    , { IIWA_CMD_CARTESIAN_POSE_LIN        , "CMD_CARTESIAN_POSE_LIN"        } 
    , { IIWA_CMD_CARTESIAN_VELOCITY        , "CMD_CARTESIAN_VELOCITY"        } 
    , { IIWA_CMD_JOINT_POSITION            , "CMD_JOINT_POSITION"            } 
    , { IIWA_CMD_JOINT_POSITION_VELOCITY   , "CMD_JOINT_POSITION_VELOCITY"   } 
    , { IIWA_CMD_JOINT_VELOCITY            , "CMD_JOINT_VELOCITY"            } };


    
//< The IIWA allows to load also different controller (again, according to the interpolator)
enum IIWA_CONTROL_TYPE  : int { IIWA_NONE_CONTROL         = -1
                              , IIWA_POSITION_CONTROL     =  iiwa_msgs::ControlMode::POSITION_CONTROL
                              , IIWA_JOINT_IMPEDANCE      =  iiwa_msgs::ControlMode::JOINT_IMPEDANCE
                              , IIWA_CARTESIAN_IMPEDANCE  =  iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE
                              , IIWA_DESIRED_FORCE        =  iiwa_msgs::ControlMode::DESIRED_FORCE
                              , IIWA_SINE_PATTERN         =  iiwa_msgs::ControlMode::SINE_PATTERN         
                              , IIWA_FRI_JOINT_POS_CONTROL=  iiwa_msgs::ControlMode::FRI_JOINT_POS_CONTROL
                              , IIWA_FRI_JOINT_IMP_CONTROL=  iiwa_msgs::ControlMode::FRI_JOINT_IMP_CONTROL
                              , IIWA_FRI_TORQUE_CONTROL   =  iiwa_msgs::ControlMode::FRI_TORQUE_CONTROL
                              , FRI_WRENCH_CONTROL        =  iiwa_msgs::ControlMode::FRI_WRENCH_CONTROL };
                          
//< Hear some interfaces by ROSCONTROL implemented in the stack
enum ROSCONTROL_INTERFACE_TYPE  : int { ROSCONTROL_NONE_INTERFACE               = -1
                                      , ROSCONTROL_POSITION_JOINT_INTERFACE     =  0
                                      , ROSCONTROL_EFFORT_JOINT_INTERFACE       =  1
                                      , ROSCONTROL_POSVELEFF_JOINT_INTERFACE    =  2
                                      , ROSCONTROL_VELOCITY_JOINT_INTERFACE     =  3 };
                                      
                                      
const static std::map< ROSCONTROL_INTERFACE_TYPE, std::string > ROSCONTROL_INTERFACE_TYPE_IDS 
    { {  ROSCONTROL_NONE_INTERFACE               , "NoneInterface"            }
    , {  ROSCONTROL_POSITION_JOINT_INTERFACE     , "PositionJointInterface"   }
    , {  ROSCONTROL_EFFORT_JOINT_INTERFACE       , "EffortJointInterface"     }
    , {  ROSCONTROL_POSVELEFF_JOINT_INTERFACE    , "PosVelEffJointInterface"  }
    , {  ROSCONTROL_VELOCITY_JOINT_INTERFACE     , "VelocityJointInterface"   } };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                                                        


  
class IIWA_HW: public itia_hardware_interface::BasicRobotHW
{

public:
  IIWA_HW( const std::vector<std::string>& joint_names );
  virtual ~IIWA_HW();
  
  
  /** 
    * \brief Initializes the IIWA device struct and all the hardware and joint limits interfaces needed.
    *  
    * A joint state handle is created and linked to the current joint state of the IIWA robot.
    * A joint position handle is created and linked  to the command joint position to send to the robot.
    */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  
  /**
   * \brief Registers the limits of the joint specified by joint_name and joint_handle. 
   * 
   * The limits are retrieved from the urdf_model.
   * Returns the joint's type, lower position limit, upper position limit, and effort limit.
   */
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const urdf::Model *const urdf_model,
                           double *const lower_limit, double *const upper_limit,
                           double *const effort_limit);
    
  virtual void read (const ros::Time& time, const ros::Duration& period);
  virtual void write(const ros::Time& time, const ros::Duration& period);
  
  virtual bool prepareSwitch( const std::list<hardware_interface::ControllerInfo>& start_list
                            , const std::list<hardware_interface::ControllerInfo>& stop_list );
  
  virtual void shutdown();

  
  /**
   * \brief Retuns the ros::Rate object to control the receiving/sending rate.
   */
  ros::Rate* getRate();
  
  /**
   * \brief Retuns the current frequency used by a ros::Rate object to control the receiving/sending rate.
   */
  double getFrequency();
  
  /**
   * \brief Set the frequency to be used by a ros::Rate object to control the receiving/sending rate.
   */
  void setFrequency(double frequency);
  
    
    /** Structure for a lbr iiwa, joint handles, etc */
    struct IIWA_device {
      /** Vector containing the name of the joints - taken from yaml file */
      std::vector<std::string>  joint_names;
      
      std::vector<double>       joint_lower_limits, /**< Lower joint limits */
                                joint_upper_limits, /**< Upper joint limits */
                                joint_effort_limits; /**< Effort joint limits */
                                
      /**< Joint state and commands */
      std::vector<double>       joint_position,
                                joint_position_prev,
                                joint_velocity,
                                joint_effort,
                                joint_position_command,
                                joint_velocity_command,
                                joint_stiffness_command,
                                joint_damping_command,
                                joint_effort_command;
      
      /** 
      * \brief Initialze vectors
      */
      void init(const int nAx = IIWA_JOINTS ) {
        joint_position.resize( nAx );
        joint_position_prev.resize( nAx );
        joint_velocity.resize( nAx );
        joint_effort.resize( nAx );
        joint_position_command.resize( nAx );
        joint_velocity_command.resize( nAx );
        joint_effort_command.resize( nAx );
        joint_stiffness_command.resize( nAx );
        joint_damping_command.resize( nAx );
        
        joint_lower_limits.resize( nAx );
        joint_upper_limits.resize( nAx );
        joint_effort_limits.resize( nAx );
      }
      
      /** 
      * \brief Reset values of the vectors
      */
      void reset() {
        for (size_t j = 0; j < joint_position.size(); ++j) {
          joint_position[j] = 0.0;
          joint_position_prev[j] = 0.0;
          joint_velocity[j] = 0.0;
          joint_effort[j] = 0.0;
          joint_position_command[j] = 0.0;
          joint_velocity_command[j]=0.0;
          joint_effort_command[j] = 0.0;
          
          // set default values for these two for now
          joint_stiffness_command[j] = 0.0;
          joint_damping_command[j] = 0.0;
        }
      }
      void resetCmd() 
      {
        for (size_t j = 0; j < joint_position.size(); ++j) {
          joint_position_command[j] = joint_position[j];
          joint_velocity_command[j]=0.0;
          joint_effort_command[j] = 0.0;
          
          // set default values for these two for now
          joint_stiffness_command[j] = 0.0;
          joint_damping_command[j] = 0.0;
        }
      }
    };
protected:
  
  ros::NodeHandle nh_;
  
  ROSCONTROL_INTERFACE_TYPE interface_;
  std::string command_type_;
  std::string movegroup_name_;
  urdf::Model urdf_model_;
  
  
  hardware_interface::JointStateInterface       state_interface_;                     /**< Interface for joint state              */
  hardware_interface::PositionJointInterface    position_interface_;                  /**< Interface for joint position control   */
  hardware_interface::VelocityJointInterface    velocity_interface_;                  /**< interface for writing velocity target  */
  hardware_interface::EffortJointInterface      effort_interface_;                    /**< Interface for joint impedance control  */
  hardware_interface::PosVelEffJointInterface   position_velocity_effort_interface_;  /**< interface for writing velocity target  */
  hardware_interface::PositionJointInterface    fir_position_interface_;              /**< Interface for joint position control   */
  hardware_interface::EffortJointInterface      fir_effort_interface_;                /**< Interface for joint impedance control  */
  
  /** Interfaces for limits */
  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  
  boost::shared_ptr<IIWA_HW::IIWA_device> device_; /**< IIWA device. */
  
  /** Objects to control send/receive rate. */
  ros::Time timer_;
  ros::Rate* loop_rate_;
  double control_frequency_;
  std::mutex mutex_;
  int fri_port_;
  std::string fri_hostname_;
  bool fri_active_;
  
  iiwa_ros::iiwaRos iiwa_ros_conn_;
  iiwa_msgs::JointPosition  joint_position_;
  iiwa_msgs::JointVelocity  joint_velocity_;
  iiwa_msgs::JointTorque    joint_torque_;
  
  iiwa_msgs::JointPosition          command_joint_position_;
  iiwa_msgs::JointVelocity          command_joint_velocity_;
  iiwa_msgs::JointPositionVelocity  command_joint_position_velocity_;
  iiwa_msgs::JointTorque            command_joint_torque_;
  
  std::vector<double>     last_joint_position_command_;
    
  ros::Duration m_duration_send_zero_with_no_control;
  ros::Time m_switch_off_control_time;
  
  unsigned int m_missing_messages;
  unsigned int m_max_missing_messages;
  
  
  
};

  
template <typename T>
void iiwaMsgsJointToVector(const iiwa_msgs::JointQuantity& ax, std::vector<T>& v) {
  v[0] = ax.a1;
  v[1] = ax.a2;
  v[2] = ax.a3;
  v[3] = ax.a4;
  v[4] = ax.a5;
  v[5] = ax.a6;
  v[6] = ax.a7;
}

template <typename T>
void vectorToIiwaMsgsJoint(const std::vector<T>& v, iiwa_msgs::JointQuantity& ax) {
  ax.a1 = v[0];
  ax.a2 = v[1];
  ax.a3 = v[2];
  ax.a4 = v[3];
  ax.a5 = v[4];
  ax.a6 = v[5];
  ax.a7 = v[6];
}
}

#endif

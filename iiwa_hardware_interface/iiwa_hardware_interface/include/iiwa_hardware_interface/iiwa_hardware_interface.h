#ifndef __ITIA_TOPIC_HARDWARE_INTERFACE__
#define __ITIA_TOPIC_HARDWARE_INTERFACE__

#include <itia_basic_hardware_interface/itia_basic_hardware_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <itia_basic_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <mutex>
// namespace hardware_interface
// {
//   /// \ref JointCommandInterface for commanding acceleration-based joints.
//   class AccelerationJointInterface : public JointCommandInterface {};
// }



namespace itia_hardware_interface
{
  enum ctrlmode { velocity, posvel, pos, none};
  
  class IiwaRobotHW: public itia_hardware_interface::BasicRobotHW
  {
  public:
    IiwaRobotHW(std::vector<std::string> joint_names);
    virtual ~IiwaRobotHW()
    {
      m_mutex.lock();
      m_mutex.unlock();
    };
    virtual void shutdown();
    virtual void read(const ros::Time& time, const ros::Duration& period);
    virtual void write(const ros::Time& time, const ros::Duration& period);
    
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) ;
    
//     virtual bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) ;
//     
//     virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
//                                const std::list<hardware_interface::ControllerInfo>& stop_list);
//     
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& stop_list);
//     
    

    
  protected:
    virtual void jointPosCallback(const iiwa_msgs::JointPositionConstPtr& msg);
    virtual void jointVelCallback(const iiwa_msgs::JointVelocityConstPtr& msg);
    virtual void jointEffCallback(const iiwa_msgs::JointTorqueConstPtr& msg);
    
    
    std::shared_ptr<ros::Subscriber> m_js_sub_pos;
    std::shared_ptr<ros::Subscriber> m_js_sub_vel;
    std::shared_ptr<ros::Subscriber> m_js_sub_eff;
    std::shared_ptr<ros::Publisher>  m_js_v_pub;
    std::shared_ptr<ros::Publisher>  m_js_p_pub;
    std::shared_ptr<ros::Publisher>  m_js_pv_pub;
    std::shared_ptr<ros::Publisher>  m_js_target_pub;
    iiwa_msgs::JointVelocityPtr m_msg; 
    
    hardware_interface::JointStateInterface    m_js_jh; //interface for reading joint state
    hardware_interface::VelocityJointInterface m_v_jh; //interface for writing velocity target
    hardware_interface::PositionJointInterface m_p_jh; //interface for writing velocity target
    hardware_interface::PosVelEffJointInterface m_pve_jh; //interface for writing velocity target
    
   ros::Duration m_duration_send_zero_with_no_control;
   ros::Time m_switch_off_control_time;
    
    std::vector<std::string> m_joint_names;
    
    
    std::vector<double> m_pos; // feedback position
    std::vector<double> m_vel; // feedback velocity
    std::vector<double> m_eff; // feedback effort
    
    std::vector<double> m_cmd_pos; //target position
    std::vector<double> m_cmd_vel; //target velocity
    std::vector<double> m_cmd_eff; //target effort
    
    
    
    unsigned int m_nAx;
    unsigned int m_missing_messages;
    unsigned int m_max_missing_messages;
    
    bool m_pos_topic_received;
    bool m_vel_topic_received;
    bool m_eff_topic_received;
    
    std::mutex m_mutex;
    ctrlmode m_ctrl_mode;
    
  };
}

#endif

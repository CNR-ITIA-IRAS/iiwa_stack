#include <iiwa_hardware_interface/iiwa_hw.h>
#include <control_toolbox/filters.h>

namespace itia_hardware_interface
{
  
size_t findCaseInsensitive(std::string data, std::string toSearch, size_t pos = 0)
{
  // Convert complete given String to lower case
  std::transform(data.begin(), data.end(), data.begin(), ::tolower);
  // Convert complete given Sub String to lower case
  std::transform(toSearch.begin(), toSearch.end(), toSearch.begin(), ::tolower);
  // Find sub string in given string
  return data.find(toSearch, pos);
}
std::string removeSubstring(const std::string& data, const std::string& toSearch)
{
  std::string ret = data;
  size_t pos = ret.find( toSearch );
  if (pos != std::string::npos)
  {
    ret.erase(pos, std::string("hardware_interface::").length());
  }
  return ret;
}

  
  
IIWA_HW::IIWA_HW( const std::vector<std::string>& joint_names )
  : last_joint_position_command_(7, 0) 
{
  timer_ = ros::Time::now();
  control_frequency_ = DEFAULT_CONTROL_FREQUENCY;
  loop_rate_ = new ros::Rate(control_frequency_);
  interface_ = ROSCONTROL_NONE_INTERFACE;
  device_.reset( new IIWA_HW::IIWA_device() );
  
  device_->joint_names = joint_names;
  if ( device_->joint_names.size() != IIWA_JOINTS) 
  {
    ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
    throw std::runtime_error("This robot has 7 joints, you must specify 7 names for each one");
  } 
  
  if (!(urdf_model_.initParam("robot_description"))) 
  {
    ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
    throw std::runtime_error("No URDF model available");
  }
}

IIWA_HW::~IIWA_HW()
{
  mutex_.lock();
  shutdown();
  mutex_.unlock();
}

bool IIWA_HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  ROS_INFO("[%s] >>> IIWA_HW Initialization started",robot_hw_nh.getNamespace().c_str());
  if (!itia_hardware_interface::BasicRobotHW::init(root_nh, robot_hw_nh))
  {
    ROS_ERROR("[%s] init() error",robot_hw_nh.getNamespace().c_str());
    return false;
  }
  ROS_INFO("[%s] Basic Robot Initialization ok",robot_hw_nh.getNamespace().c_str());
  
  
  
  // TODO: use transmission configuration to get names directly from the URDF model
  if ( robot_hw_nh.getParam("fri_port", fri_port_) )
  {
    fri_port_ = -1;
  } 
  if ( robot_hw_nh.getParam("fri_hostname", fri_hostname_) )
  {
    fri_hostname_ = "n/a";
  }
  if ( robot_hw_nh.getParam("control_frequency_", control_frequency_) )
  {
    fri_hostname_ = "n/a";
  }

  ROS_INFO("[%s] iiwa ros connection initialization ",m_robot_hw_nh.getNamespace().c_str());
  iiwa_ros_conn_.init(1./control_frequency_, true);
  
  // initialize and set to zero the state and command values
  ROS_INFO("[%s] >>> iiwa ros connection initialization ",m_robot_hw_nh.getNamespace().c_str());
  device_->init( device_->joint_names.size() );
  device_->reset( );
  
  // general joint to store information
  std::shared_ptr<const urdf::Joint> joint;
  
  // create joint handles given the list
  for(size_t i = 0; i < device_->joint_names.size(); ++i) 
  {
    ROS_INFO_STREAM("Handling joint: " << device_->joint_names[i]);
    
    // get current joint configuration
    joint = urdf_model_.getJoint(device_->joint_names[i]);
    if(!joint.get()) {
      ROS_ERROR_STREAM("The specified joint "<< device_->joint_names[i] << " can't be found in the URDF model. "
      "Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
      throw std::runtime_error("Wrong joint name specification");
    }
    
    { // joint state handle
      hardware_interface::JointStateHandle  state_handle(   device_->joint_names[i]
                                                        , &(device_->joint_position[i])
                                                        , &(device_->joint_velocity[i])
                                                        , &(device_->joint_effort[i]));
      
      state_interface_.registerHandle(state_handle);
    }
    
    
    
    { // position command handle
      hardware_interface::JointHandle position_joint_handle ( state_interface_.getHandle(device_->joint_names[i])
                                                            , &device_->joint_position_command[i]);
      
      position_interface_.registerHandle(position_joint_handle);
    }
    
    
    
    { // velocity command handle
      hardware_interface::JointHandle velocity_joint_handle ( state_interface_.getHandle(device_->joint_names[i])
                                                            , &device_->joint_velocity_command[i]);
      
      velocity_interface_.registerHandle(velocity_joint_handle);
    }
    
    
    { // effort command handle
      hardware_interface::JointHandle joint_handle( state_interface_.getHandle(device_->joint_names[i])
                                                  , &device_->joint_effort_command[i] );
            
      effort_interface_.registerHandle(joint_handle);
    }
    
    
    { // effort command handle
      hardware_interface::PosVelEffJointHandle joint_handle( state_interface_.getHandle(device_->joint_names[i])
                                                          , &device_->joint_position_command[i]
                                                          , &device_->joint_velocity_command[i]
                                                          , &device_->joint_effort_command  [i]);
      
      
      position_velocity_effort_interface_.registerHandle(joint_handle);
    }
    
    
    {
      registerJointLimits(device_->joint_names[i],
                          position_interface_.getHandle(device_->joint_names[i]),
                          &urdf_model_,
                          &device_->joint_lower_limits[i],
                          &device_->joint_upper_limits[i],
                          &device_->joint_effort_limits[i] );
    }
  }
  
  // TODO: CHECK
  // register ros-controls interfaces
  ROS_INFO("[ %s ] Register state interface",m_robot_hw_nh.getNamespace().c_str());
  this->registerInterface(&state_interface_);
  ROS_INFO("[ %s ] Register position velocity effort interface",m_robot_hw_nh.getNamespace().c_str());
  this->registerInterface(&position_velocity_effort_interface_);
  ROS_INFO("[ %s ] Register effort interface",m_robot_hw_nh.getNamespace().c_str());
  this->registerInterface(&effort_interface_);
  ROS_INFO("[ %s ] Register velocity interface",m_robot_hw_nh.getNamespace().c_str());
  this->registerInterface(&velocity_interface_);
  ROS_INFO("[ %s ] Register position interface",m_robot_hw_nh.getNamespace().c_str());
  this->registerInterface(&position_interface_);

  m_duration_send_zero_with_no_control = ros::Duration(0.1);
  m_switch_off_control_time            = ros::Time::now();
  
  
  return true;
}




bool IIWA_HW::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  ROS_INFO("[ %s ] Prepare Switch",m_robot_hw_nh.getNamespace().c_str());
  if(start_list.size()==0) 
  {
    if( stop_list.size()==0) 
    {
      ROS_FATAL("[ %s ] NO CONTROLLERS NETIHER TO LOAD AND UNLOAD",m_robot_hw_nh.getNamespace().c_str());
    }
    else
    {
      ROS_FATAL("[ %s ] NO CONTROLLERS, PUBLISHING ZERO VELOCITY FOR %f second",m_robot_hw_nh.getNamespace().c_str(),m_duration_send_zero_with_no_control.toSec());
    }
    interface_ = ROSCONTROL_NONE_INTERFACE;
    m_switch_off_control_time = ros::Time::now();
    return false;
  }
  else if ( start_list.size() > 1)
  {
    ROS_FATAL("[ %s ] The HW allow the loading and unliading only of one controller at time",m_robot_hw_nh.getNamespace().c_str());
    return false;
  }
  
  if( start_list.front().claimed_resources.size() == 0 )
  {
    ROS_FATAL("[ %s] NO RESOURCES ASSOCIATED TO THE CONTROLLER",m_robot_hw_nh.getNamespace().c_str());
    return false;
  }
  else if( start_list.front().claimed_resources.size() > 1 )
  {
    ROS_FATAL("[ %s ] ONLY ONE RESOURCE AT TIME (e.g., all the joints togheter) CAN BE ASSOCIATED TO THE CONTROLLER",m_robot_hw_nh.getNamespace().c_str());
    return false;
  }

  
  const hardware_interface::ControllerInfo&       controller = start_list.front();
  const hardware_interface::InterfaceResources&   resource   = controller.claimed_resources[0];
  
  bool is_fri_controller = ( findCaseInsensitive(controller.name, "FRI") != std::string::npos );
  if( is_fri_controller )
  { 
    if( fri_port_ == -1 )
    {
      ROS_ERROR("[ %s ]A FIR Controller is loaded, but the 'fri_port' and 'fri_hostname' are not in the parameter server. Abort. ",m_robot_hw_nh.getNamespace().c_str());
      return false;
    }
  }
    
  std::string interface_str = removeSubstring( resource.hardware_interface, "hardware_interface::");
    
  auto it = ROSCONTROL_INTERFACE_TYPE_IDS.begin(); 
  for( it = ROSCONTROL_INTERFACE_TYPE_IDS.begin(); it!=ROSCONTROL_INTERFACE_TYPE_IDS.end(); it++)
  {
    if( interface_str.compare( it->second ) == 0 )
      break;
  }
  if( it == ROSCONTROL_INTERFACE_TYPE_IDS.end() )
  {
    ROS_ERROR("Error in the configuration. None valid interface with ID '%s' is in the list of the allowed ones. Abort. ", ROSCONTROL_INTERFACE_TYPE_IDS.at( interface_ ).c_str());
    return false;
  }
  
  bool ret = false;
  interface_ = it->first;
  switch( interface_ ) 
  {
    case ROSCONTROL_NONE_INTERFACE : 
      ROS_ERROR("Error in the configuration. None valid interface with ID '%s' is in the list of the allowed ones. Abort. ", ROSCONTROL_INTERFACE_TYPE_IDS.at( interface_ ).c_str());
      ret = iiwa_ros_conn_.getServoMotion().setPositionControlMode(); //for safety
      ret = false;
      break;
    case ROSCONTROL_POSITION_JOINT_INTERFACE : 
      ret = iiwa_ros_conn_.getServoMotion().setPositionControlMode();
      break;
    case ROSCONTROL_EFFORT_JOINT_INTERFACE : 
      ROS_ERROR("ROSCONTROL_EFFORT_JOINT_INTERFACE to be implemented. ");
      ret = false;
      break;
    case ROSCONTROL_POSVELEFF_JOINT_INTERFACE :
      ret = is_fri_controller 
          ? ( fri_active_ = iiwa_ros_conn_.getServoMotion().setFRIJointPositionControlMode( ) )
          : iiwa_ros_conn_.getServoMotion().setPositionControlMode();
      break;
    case ROSCONTROL_VELOCITY_JOINT_INTERFACE : 
      ret = iiwa_ros_conn_.getServoMotion().setPositionControlMode();
      break;
    default:
      ROS_ERROR("Why in Default state? what wrong I did? ");
      ret = false;
      break;
  }
  ROS_INFO("[ %s ] Prepare Switch %s",m_robot_hw_nh.getNamespace().c_str(), ( (ret) ? "SUCCESS" : "FAILURE" ) );
  return( ret );
}



void IIWA_HW::read(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();

  ros::Duration delta = ros::Time::now() - timer_;
  
  static bool was_connected = false;
  
  if (iiwa_ros_conn_.getRobotIsConnected()) {
    
    iiwa_ros_conn_.getJointPosition(joint_position_);
    iiwa_ros_conn_.getJointVelocity(joint_velocity_);
    iiwa_ros_conn_.getJointTorque  (joint_torque_  );
    
    device_->joint_position_prev = device_->joint_position;
    iiwaMsgsJointToVector(joint_position_.position, device_->joint_position);
    iiwaMsgsJointToVector(joint_velocity_.velocity, device_->joint_velocity);
    iiwaMsgsJointToVector(joint_torque_  .torque  , device_->joint_effort  );
    
    // if there is no controller active the robot goes to zero position 
    if (!was_connected) {
      for (size_t j = 0; j < device_->joint_names.size(); j++)
      {
        device_->reset();
        device_->joint_position_command[j] = device_->joint_position[j];
      }
      
      was_connected = true;
    }
    
    for (size_t j = 0; j < device_->joint_names.size(); j++)
      device_->joint_velocity[j] = filters::exponentialSmoothing((device_->joint_position[j]-device_->joint_position_prev[j])/period.toSec(), 
                                                                  device_->joint_velocity[j], 0.2);  
      
  } else if (delta.toSec() >= 10) {
    ROS_INFO("No LBR IIWA is connected. Waiting for the robot to connect before reading ...");
    timer_ = ros::Time::now();
  }
  return;
}

void IIWA_HW::write(const ros::Time& time, const ros::Duration& period)
{
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  
  ros::Duration delta = ros::Time::now() - timer_;
  
  //reading the force/torque values
  if (iiwa_ros_conn_.getRobotIsConnected()) 
  {
    
    switch( interface_ ) 
    {
      
      case ROSCONTROL_NONE_INTERFACE : 
      {
        vectorToIiwaMsgsJoint(device_->joint_position, command_joint_position_.position);
        iiwa_ros_conn_.setJointPosition(command_joint_position_);
      }
      break;
      /**
       * 
       */
      case ROSCONTROL_POSITION_JOINT_INTERFACE : 
        if ( fri_active_ || (device_->joint_position_command != last_joint_position_command_)  ) // avoid sending the same joint command over and over
        {
          last_joint_position_command_ = device_->joint_position_command;
            
          // Building the message
          vectorToIiwaMsgsJoint(device_->joint_position_command, command_joint_position_.position);
          command_joint_position_.header.stamp = ros::Time::now();
            
          iiwa_ros_conn_.setJointPosition(command_joint_position_);
        }
        break;
      case ROSCONTROL_EFFORT_JOINT_INTERFACE : 
      {
        throw std::runtime_error( "TO IMPLEMENT" );
      }
      break;
      case ROSCONTROL_POSVELEFF_JOINT_INTERFACE : 
        vectorToIiwaMsgsJoint(device_->joint_position_command, command_joint_position_velocity_.position);
        vectorToIiwaMsgsJoint(device_->joint_velocity_command, command_joint_position_velocity_.velocity);
        command_joint_position_velocity_.header.stamp = ros::Time::now();
          
        iiwa_ros_conn_.setJointPositionVelocity(command_joint_position_velocity_);
      
      break;
      case ROSCONTROL_VELOCITY_JOINT_INTERFACE : 
      {
        vectorToIiwaMsgsJoint(device_->joint_velocity_command, command_joint_velocity_.velocity);
        command_joint_velocity_.header.stamp = ros::Time::now();
            
        iiwa_ros_conn_.setJointVelocity(command_joint_velocity_);
        break;
      }
      break;
    }
  } 
  else if (delta.toSec() >= 10) 
  {
    ROS_INFO_STREAM("No LBR IIWA is connected. Waiting for the robot to connect before writing ...");
    timer_ = ros::Time::now();
  }
  
  return ;
}


void IIWA_HW::shutdown()
{
  mutex_.lock();  
  device_->resetCmd();
  mutex_.unlock();
  
}


ros::Rate* IIWA_HW::getRate() {
  return loop_rate_;
}

double IIWA_HW::getFrequency() {
  return control_frequency_;
}

void IIWA_HW::setFrequency(double frequency) {
  control_frequency_ = frequency;
  loop_rate_ = new ros::Rate(control_frequency_);
}




void IIWA_HW::registerJointLimits ( const std::string&                      joint_name
                                  , const hardware_interface::JointHandle&  joint_handle
                                  , const urdf::Model *const                urdf_model
                                  , double *const                           lower_limit
                                  , double *const                           upper_limit
                                  , double *const                           effort_limit ) 
{
  
  *lower_limit  = -std::numeric_limits<double>::max();
  *upper_limit  =  std::numeric_limits<double>::max();
  *effort_limit =  std::numeric_limits<double>::max();
  
  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;
  
  if (urdf_model != NULL) {
    const std::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
    
    if (urdf_joint != NULL) {
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  
  if (!has_limits)
    return;
  
  if (limits.has_position_limits) {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;
  
  if (has_soft_limits) {
    const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
    ej_limits_interface_.registerHandle(limits_handle);
  } else {
    const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
    ej_sat_interface_.registerHandle(sat_handle);
  }
}




}


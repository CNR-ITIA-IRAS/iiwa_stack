#include <iiwa_hi_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::IiwaHwIfaceNodelet, nodelet::Nodelet) 

namespace itia
{
  namespace control
  {
    void IiwaHwIfaceNodelet::onInit()
    {
      
      m_console_name = getPrivateNodeHandle().getNamespace()+" type: IiwaHwIfaceNodelet";
      
      ROS_INFO("[%s] STARTING", m_console_name.c_str());
      m_stop = false;
      
      std::vector<std::string> joint_names;
      if (!getPrivateNodeHandle().getParam("joint_names", joint_names))
      {
        ROS_FATAL_STREAM(getPrivateNodeHandle().getNamespace()+"/joint_names' does not exist");
        ROS_FATAL("ERROR DURING STARTING HARDWARE INTERFACE '%s'", getPrivateNodeHandle().getNamespace().c_str());
        return;
      }
      ros::NodeHandle nh;
      m_hw.reset(new iiwa_hw::IiwaHw(nh));
      m_main_thread = std::thread(&itia::control::IiwaHwIfaceNodelet::mainThread, this);
      
    };
    
  }
}

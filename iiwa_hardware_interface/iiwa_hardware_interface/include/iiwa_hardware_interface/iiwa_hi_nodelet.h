#ifndef __TOPIC_HARDWARE_INTERFACE__ON_NODELET__
#define __TOPIC_HARDWARE_INTERFACE__ON_NODELET__

#include <controller_manager/controller_manager.h>
#include <nodelet/nodelet.h>
#include <thread>
#include <iiwa_hardware_interface/iiwa_hw.h>
#include <itia_basic_hardware_interface/basic_hi_nodelet.h>

namespace itia
{
  namespace control
  {
    
    class IiwaHwIfaceNodelet : public BasicHwIfaceNodelet
    {
    public:
      virtual void onInit();
      
    protected:
    };
    
    
    
  }
}
# endif
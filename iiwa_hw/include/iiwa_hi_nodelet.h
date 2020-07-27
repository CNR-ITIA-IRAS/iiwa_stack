#pragma once

# include <controller_manager/controller_manager.h>
# include <nodelet/nodelet.h>
# include <thread>
# include <iiwa_hw.h>
# include <cnr_hardware_interface/basic_hi_nodelet.h>
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


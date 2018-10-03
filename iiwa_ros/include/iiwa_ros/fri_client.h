#pragma once

#ifdef ENABLE_FRI

#include <ros/ros.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_fri/friUdpConnection.h>
#include <iiwa_fri/friClientApplication.h>
#include <iiwa_fri/friLBRClient.h>
#include <realtime_utilities/circular_buffer.h>

 
template< typename T > std::string to_string  ( const std::vector< T >& v
                                              , const std::string prefix = "["
                                              , const std::string delimeter = ", "
                                              , const std::string terminator ="]" )
{
  std::string ret = prefix;
  if(v.size() == 0)
    return "";
  
  for( size_t i=0; i < v.size()-1; i++)
    ret += std::to_string( v[i] ) + delimeter;
  ret += std::to_string( v.back() ) + terminator;
  
  return ret;
}


namespace iiwa_ros 
{
  
class LBRJointOverlayClient : public KUKA::FRI::LBRClient
{
  public:
  
    LBRJointOverlayClient( const size_t target_queue_lenght = 1e3 ) 
      : command_joint_position_( target_queue_lenght ) 
      , last_joint_pos_command_( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , initial_joints_        ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , control_running_       ( false )
    { 
      // nothing to do so far;
    };
    
    ~LBRJointOverlayClient( ) { };
    
    virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
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
          break;
        case KUKA::FRI::COMMANDING_WAIT:
          initial_joints_ = std::vector<double>( robotState().getIpoJointPosition(), robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS );
          last_joint_pos_command_ = initial_joints_;
          break;
          
        case KUKA::FRI::COMMANDING_ACTIVE:
          initial_joints_ = std::vector<double>( robotState().getIpoJointPosition(), robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS );
          control_running_ = true;
          break;
      }  
      actual_state_prev_ = oldState;
      actual_state_ = newState;
    }
    
    virtual void command()
    { 
      if( ((int)actual_state_prev_ > (int)KUKA::FRI::IDLE) && (actual_state_ == KUKA::FRI::IDLE) )
      {
        return;
      }
        
      if( command_joint_position_.empty() )
      {
        
        //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
        robotCommand().setJointPosition( &last_joint_pos_command_[0] );
      }
      else
      {
        //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
        last_joint_pos_command_ = command_joint_position_.front();
        robotCommand().setJointPosition( &last_joint_pos_command_[0] );
        command_joint_position_.pop_front();
      }
    }
    
    void newJointPosCommand( const std::vector< double >& new_joint_pos_command ) 
    {
      if( new_joint_pos_command.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS )
      {
        throw std::runtime_error("Mismatch of New Command Vector"); 
      }
      command_joint_position_.push_back( new_joint_pos_command );
      return;
    }
    
    void getState(KUKA::FRI::ESessionState& oldState, KUKA::FRI::ESessionState& newState) const
    {
      oldState = actual_state_prev_;
      newState = actual_state_;
    }
    
    bool isControlRunning( ) const { return control_running_; }
        
  private:
    realtime_utilities::circ_buffer< std::vector<double> > command_joint_position_;
    std::vector<double> last_joint_pos_command_;
    std::vector<double> initial_joints_;
    KUKA::FRI::ESessionState actual_state_;
    KUKA::FRI::ESessionState actual_state_prev_;
    bool control_running_;
};
  
    

  

  
class LBRJointOverlayApp
{
public:
  
  
  LBRJointOverlayApp( const int& port, const std::string& hostname ) 
    : port_      ( port )
    , hostname_  ( hostname )
    , connection_( 500 )
  {
    connect();  
  }
    
  ~LBRJointOverlayApp( )
  {
    ROS_INFO("Destroy the JointOverlayApp (disconnect the FRI)");
    disconnect();
  }

  void newJointPosCommand( const iiwa_msgs::JointPosition& position ) 
  {  
    std::vector<double> new_joint_pos_command( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 );
    new_joint_pos_command[0] = position.position.a1;
    new_joint_pos_command[1] = position.position.a2;
    new_joint_pos_command[2] = position.position.a3;
    new_joint_pos_command[3] = position.position.a4;
    new_joint_pos_command[4] = position.position.a5;
    new_joint_pos_command[5] = position.position.a6;
    new_joint_pos_command[6] = position.position.a7;
    return client_.newJointPosCommand( new_joint_pos_command );
  }


  void connect()
  {
    ROS_WARN("FRI Connect to %s:%d ", hostname_.c_str(), port_ );
    app_.reset(new KUKA::FRI::ClientApplication(connection_, client_) );
    if( !app_->connect(port_, hostname_.size() > 0 ? hostname_.c_str() : nullptr ) )
    {
      throw std::runtime_error( ("Error in FRI Connection (" + hostname_ + ":" + std::to_string( port_ ) ).c_str() );
    }
    stop_fri_command_thread_ = false;
    command_thread_.reset( new boost::thread(boost::bind(&LBRJointOverlayApp::step, this)) );    
    ROS_WARN("FRI Connect. Done.");
  }
  void disconnect()
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

private:
  
  void step()
  {
    bool success = true;
    while( (!stop_fri_command_thread_) && ros::ok() && success )
    {
      if( app_ )
      {
        
//         KUKA::FRI::ESessionState oldState, newState;
//         client_.getState(oldState,newState);
//         if( client_.isControlRunning() && ((int)oldState > (int)KUKA::FRI::IDLE) && ( newState == KUKA::FRI::IDLE ) )
//         {
//           ROS_WARN("Break the FRI control loop. Shutdown transition detected.(%d/%d)", (int) oldState, (int)newState );
//           break;
//         }

        success = app_->step();
      }
      else
      {
        ROS_INFO("App reset. Exit. ");
        break;
      }
    }
  }
  
  bool                                            stop_fri_command_thread_;
  std::shared_ptr<boost::thread>                  command_thread_; 
  int                                             port_;
  std::string                                     hostname_;
  KUKA::FRI::UdpConnection                        connection_;
  LBRJointOverlayClient                           client_;
  std::shared_ptr< KUKA::FRI::ClientApplication > app_;


};


}


#endif
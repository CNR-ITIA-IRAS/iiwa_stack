#pragma once

#ifdef ENABLE_FRI

#include <ros/ros.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/CartesianQuantity.h>
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
      : command_joint_position_         ( target_queue_lenght ) 
      , command_joint_torque_           ( 100 )
      , command_wrench_                 ( 100 ) 
      , last_joint_pos_command_         ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , last_joint_torque_command_      ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , initial_joints_                 ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , last_wrench_command_            ( 6, 0.0 )
      , initial_wrench_                 ( 6, 0.0 )
      , control_running_                ( false )
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
          initial_torque_ = std::vector<double>( 7, 0 );
          last_joint_torque_command_ = initial_torque_;
          initial_wrench_ = std::vector<double>( 7, 0 );
          last_wrench_command_ = initial_wrench_;
          break;
          
        case KUKA::FRI::COMMANDING_WAIT:
          initial_joints_ = std::vector<double>( robotState().getIpoJointPosition(), robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS );
          last_joint_pos_command_ = initial_joints_;
          initial_torque_ = std::vector<double>( 7, 0 );
          last_joint_torque_command_ = initial_torque_;
          initial_wrench_ = std::vector<double>( 7, 0 );
          last_wrench_command_ = initial_wrench_;
          break;
          
        case KUKA::FRI::COMMANDING_ACTIVE:
          initial_joints_ = std::vector<double>( robotState().getIpoJointPosition(), robotState().getIpoJointPosition() + KUKA::FRI::LBRState::NUMBER_OF_JOINTS );
          initial_torque_ = std::vector<double>( 7, 0 );
          control_running_ = true;
          initial_wrench_ = std::vector<double>( 7, 0 );
          break;
      }  
      actual_state_prev_ = oldState;
      actual_state_ = newState;
    }
    
    void waitForCommand()
    {
        // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
        // the base method.
        LBRClient::waitForCommand();
        
        LBRClient::command();
        
        // If we want to command torques, we have to command them all the time; even in
        // waitForCommand(). This has to be done due to consistency checks. In this state it is 
        // only necessary, that some torque values are sent. The LBR does not take the 
        // specific value into account.
        if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE)
        {
            robotCommand().setTorque( &last_joint_pos_command_[0] );
        }
        else if (robotState().getClientCommandMode() == KUKA::FRI::WRENCH)
        {
            robotCommand().setWrench( &last_wrench_command_[0] );
        }
    }
      
    virtual void command()
    {
      // In command(), the joint values have to be sent. Which is done by calling
      // the base method.
      LBRClient::command();

      if( robotState().getClientCommandMode() == KUKA::FRI::POSITION )
      {
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
      else if( robotState().getClientCommandMode() == KUKA::FRI::TORQUE )
      {
        if( command_joint_torque_.empty() )
        {
          std::vector<double> zeroTorque(7,0);
          robotCommand().setTorque( &zeroTorque[0] );
        }
        else
        {
          //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
          last_joint_torque_command_ = command_joint_torque_.front();
          robotCommand().setTorque( &last_joint_torque_command_[0] );
          command_joint_torque_.pop_front();
        }
      }
      else if ( robotState().getClientCommandMode() == KUKA::FRI::WRENCH )
      {
        if( command_wrench_.empty() )
        {
          std::vector<double> zeroWrench(6,0);
          robotCommand().setWrench( &zeroWrench[0] );
        }
        else
        {
          last_wrench_command_ = command_wrench_.front();
          robotCommand().setWrench( &last_wrench_command_[0] );
          command_wrench_.pop_front();
        } 
      }
    }
    
    void newJointPosCommand( const std::vector< double >& new_joint_pos_command ) 
    {
      while( robotState().getSessionState() != KUKA::FRI::COMMANDING_ACTIVE )
      {
        ros::Duration(0.005).sleep();
      }
      
      if( robotState().getClientCommandMode() != KUKA::FRI::POSITION )
      {
        ROS_WARN_STREAM(ros::Time::now() << " fri_client.h - wrong command state in newJointPosCommand ");
        return;
      }

      if( new_joint_pos_command.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS )
      {
        throw std::runtime_error("Mismatch of New Command Vector"); 
      }
      command_joint_position_.push_back( new_joint_pos_command );
      return;
    }
    
    void newJointTorqueCommand( const std::vector< double >& new_joint_torque_command ) 
    {
      while( robotState().getSessionState() != KUKA::FRI::COMMANDING_ACTIVE )
      {
        ros::Duration(0.005).sleep();
      }
      
      if( robotState().getClientCommandMode() != KUKA::FRI::TORQUE )
      {
        ROS_WARN_STREAM(ros::Time::now() << " fri_client.h - wrong command state in newJointTorqueCommand ");
        return;
      }
        
      if( new_joint_torque_command.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS )
      {
        throw std::runtime_error("Mismatch of New Command Vector"); 
      }
      command_joint_torque_.push_back( new_joint_torque_command );
    
      return;
    }
    
    void newWrenchCommand( const std::vector< double >& new_wrench_command ) 
    {
      while( robotState().getSessionState() != KUKA::FRI::COMMANDING_ACTIVE )
      {
        ros::Duration(0.005).sleep();
      }
      
      if( robotState().getClientCommandMode() != KUKA::FRI::WRENCH )
      {
        ROS_WARN_STREAM(ros::Time::now() << " fri_client.h - wrong command state in newWrenchCommand ");
        return;
      }
        
      if( new_wrench_command.size() != 6 )
      {
        throw std::runtime_error("Mismatch of New Command Vector"); 
      }
      command_wrench_.push_back( new_wrench_command );
    
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
    realtime_utilities::circ_buffer< std::vector<double> > command_joint_torque_;
    realtime_utilities::circ_buffer< std::vector<double> > command_wrench_;
    std::vector<double> last_joint_pos_command_;
    std::vector<double> last_joint_torque_command_;
    std::vector<double> last_wrench_command_;
    std::vector<double> initial_joints_;
    std::vector<double> initial_torque_;
    std::vector<double> initial_wrench_;
    KUKA::FRI::ESessionState actual_state_;
    KUKA::FRI::ESessionState actual_state_prev_;
    bool control_running_;
};

class LBROverlayApp
{
public:
  
  
  LBROverlayApp( const int& port, const std::string& hostname, int connection_timeout_ms ) 
    : port_      ( port )
    , hostname_  ( hostname )
    , connection_( connection_timeout_ms )
    , client_    (  )
  {
    connect();  
  }
    
  ~LBROverlayApp( )
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
  
  void newJointTorqueCommand( const iiwa_msgs::JointTorque& torque ) 
  {  
    std::vector<double> new_joint_torque_command( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 );
    new_joint_torque_command[0] = torque.torque.a1;
    new_joint_torque_command[1] = torque.torque.a2;
    new_joint_torque_command[2] = torque.torque.a3;
    new_joint_torque_command[3] = torque.torque.a4;
    new_joint_torque_command[4] = torque.torque.a5;
    new_joint_torque_command[5] = torque.torque.a6;
    new_joint_torque_command[6] = torque.torque.a7;
    return client_.newJointTorqueCommand( new_joint_torque_command );
  }

  void newWrenchCommand( const iiwa_msgs::CartesianQuantity& wrench ) 
  {  
    std::vector<double> new_wrench_command( 6, 0.0 );
    new_wrench_command[0] = wrench.x;
    new_wrench_command[1] = wrench.y;
    new_wrench_command[2] = wrench.z;
    new_wrench_command[3] = wrench.a;
    new_wrench_command[4] = wrench.b;
    new_wrench_command[5] = wrench.c;
    return client_.newWrenchCommand( new_wrench_command );
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
    command_thread_.reset( new boost::thread(boost::bind(&LBROverlayApp::step, this)) );    
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
#pragma once

#ifdef ENABLE_FRI

#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

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

inline double diff  ( const std::vector< double >& v1
                    , const std::vector< double >& v2 )
{
  assert(v1.size()==v2.size() );
  assert(v1.size()>0);
  
  std::vector < double >  diff;
  
  for( size_t i=0; i< v1.size(); i++ )
      diff.push_back(std::fabs( v1[i] - v2[i] ) );
  
  return *std::max_element( diff.begin(), diff.end() );
}

namespace iiwa_ros 
{
class LBRJointOverlayClient : public KUKA::FRI::LBRClient
{
  public:
  
    LBRJointOverlayClient( const size_t target_queue_lenght = 1e8 ) 
      : command_joint_position_         ( target_queue_lenght ) 
      , command_joint_torque_           ( target_queue_lenght )
      , command_wrench_                 ( target_queue_lenght ) 
      , last_joint_pos_command_         ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , last_joint_torque_command_      ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , initial_joints_                 ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , joint_pos_command_              ( KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0 )
      , last_wrench_command_            ( 6, 0.0 )
      , initial_wrench_                 ( 6, 0.0 )
      , control_running_                ( false )
    { 
      
      realtime_pub_.init( ros::NodeHandle("~"), "fri", 100);
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
            robotCommand().setTorque( &last_joint_torque_command_[0] );
        }
        else if (robotState().getClientCommandMode() == KUKA::FRI::WRENCH)
        {
            robotCommand().setWrench( &last_wrench_command_[0] );
        }
    }
      
    virtual void command()
    {
      joint_pos_command_ = last_joint_pos_command_;
      if( robotState().getClientCommandMode() == KUKA::FRI::POSITION )
      {
        if( command_joint_position_.empty() )
        {
          
          //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
          robotCommand().setJointPosition( &joint_pos_command_[0] );
        }
        else
        {
          joint_pos_command_ = command_joint_position_.front();
          robotCommand().setJointPosition( &joint_pos_command_[0] );
          command_joint_position_.pop_front();
        }
      }
      else if( robotState().getClientCommandMode() == KUKA::FRI::TORQUE )
      {
        LBRClient::command();
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
        LBRClient::command();
        if( command_wrench_.empty() )
        {
          std::vector<double> zeroWrench(6,0);
          robotCommand().setWrench( &last_wrench_command_[0] );
        }
        else
        {
          last_wrench_command_ = command_wrench_.front();
          robotCommand().setWrench( &last_wrench_command_[0] );
          command_wrench_.pop_front();
        }
        
      }
      
      auto const joint_pos_msr = robotState().getMeasuredJointPosition();
      auto const joint_pos_cmd_iiwa = robotState().getCommandedJointPosition();
      
      if (realtime_pub_.trylock())
      {
        realtime_pub_.msg_.name = {"a1_cmd", "a2_cmd", "a3_cmd", "a4_cmd","a5_cmd", "a6_cmd", "a7_cmd"
                                  ,"a1_msr", "a2_msr", "a3_msr", "a4_msr","a5_msr", "a6_msr", "a7_msr" 
                                  ,"a1_cmd_iiwa", "a2_cmd_iiwa", "a3_cmd_iiwa", "a4_cmd_iiwa","a5_cmd_iiwa", "a6_cmd_iiwa", "a7_cmd_iiwa" };
        realtime_pub_.msg_.position = {last_joint_pos_command_[0], last_joint_pos_command_[1], last_joint_pos_command_[2]
                                     , last_joint_pos_command_[3], last_joint_pos_command_[4], last_joint_pos_command_[5], last_joint_pos_command_[6]
                                     , joint_pos_msr[0], joint_pos_msr[1], joint_pos_msr[2]
                                     , joint_pos_msr[3], joint_pos_msr[4], joint_pos_msr[5], joint_pos_msr[6] 
                                     , joint_pos_cmd_iiwa[0], joint_pos_cmd_iiwa[1], joint_pos_cmd_iiwa[2]
                                     , joint_pos_cmd_iiwa[3], joint_pos_cmd_iiwa[4], joint_pos_cmd_iiwa[5], joint_pos_cmd_iiwa[6] };
        realtime_pub_.unlockAndPublish();
      }
      last_joint_pos_command_ = joint_pos_command_ ;
    }
    
//     virtual void command()
//     {
//       // the base method.
//       
//       auto const joint_pos_msr = robotState().getMeasuredJointPosition();
//       auto const joint_pos_cmd_iiwa = robotState().getCommandedJointPosition();
//       
//       if (realtime_pub_.trylock())
//       { 
//         realtime_pub_.msg_.name     = {"a1_cmd", "a2_cmd", "a3_cmd", "a4_cmd","a5_cmd", "a6_cmd", "a7_cmd"
//                                       ,"a1_msr", "a2_msr", "a3_msr", "a4_msr","a5_msr", "a6_msr", "a7_msr" 
//                                       ,"a1_cmd_iiwa", "a2_cmd_iiwa", "a3_cmd_iiwa", "a4_cmd_iiwa","a5_cmd_iiwa", "a6_cmd_iiwa", "a7_cmd_iiwa" };
//         realtime_pub_.msg_.position = {last_joint_pos_command_[0], last_joint_pos_command_[1], last_joint_pos_command_[2]
//                                      , last_joint_pos_command_[3], last_joint_pos_command_[4], last_joint_pos_command_[5], last_joint_pos_command_[6]
//                                      , joint_pos_msr[0], joint_pos_msr[1], joint_pos_msr[2]
//                                      , joint_pos_msr[3], joint_pos_msr[4], joint_pos_msr[5], joint_pos_msr[6]
//                                      , joint_pos_cmd_iiwa[0], joint_pos_cmd_iiwa[1], joint_pos_cmd_iiwa[2]
//                                      , joint_pos_cmd_iiwa[3], joint_pos_cmd_iiwa[4], joint_pos_cmd_iiwa[5], joint_pos_cmd_iiwa[6] };
//         
//         realtime_pub_.msg_.header.stamp = ros::Time::now();
//         
//         realtime_pub_.unlockAndPublish();
//       }
// 
//       if( robotState().getClientCommandMode() == KUKA::FRI::POSITION )
//       {
//         if( command_joint_position_.empty() )
//         {
//           
//           //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
//           robotCommand().setJointPosition( &last_joint_pos_command_[0] );
//         }
//         else
//         {
//           //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
//           last_joint_pos_command_ = command_joint_position_.front();
//           robotCommand().setJointPosition( &last_joint_pos_command_[0] );
//           command_joint_position_.pop_front();
//         }
//       }
//       else if( robotState().getClientCommandMode() == KUKA::FRI::TORQUE )
//       {
//         LBRClient::command();
//         if( command_joint_torque_.empty() )
//         {
//           std::vector<double> zeroTorque(7,0);
//           robotCommand().setTorque( &zeroTorque[0] );
//         }
//         else
//         {
//           //std::cout << to_string( last_joint_pos_command_ ) << std::endl;
//           last_joint_torque_command_ = command_joint_torque_.front();
//           robotCommand().setTorque( &last_joint_torque_command_[0] );
//           command_joint_torque_.pop_front();
//         }
//       }
//       else if ( robotState().getClientCommandMode() == KUKA::FRI::WRENCH )
//       {
//         LBRClient::command();
//         if( command_wrench_.empty() )
//         {
//           std::vector<double> zeroWrench(6,0);
//           robotCommand().setWrench( &zeroWrench[0] );
//         }
//         else
//         {
//           last_wrench_command_ = command_wrench_.front();
//           robotCommand().setWrench( &last_wrench_command_[0] );
//           command_wrench_.pop_front();
//         } 
//       }
//     }
    
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
        if( command_joint_position_.empty() )
        {
          command_joint_position_.push_back( new_joint_pos_command );
        }
        else
        {
          std::vector<double> command_joint_position = command_joint_position_.back();
          if( diff( command_joint_position, new_joint_pos_command ) > 0.001 * M_PI / 180.0 )
          {
            command_joint_position_.push_back( new_joint_pos_command );
          }
        }
        return;
      }
    
//     void newJointPosCommand( const std::vector< double >& new_joint_pos_command ) 
//     {
//       while( robotState().getSessionState() != KUKA::FRI::COMMANDING_ACTIVE )
//       {
//         ros::Duration(0.005).sleep();
//       }
//       
//       if( robotState().getClientCommandMode() != KUKA::FRI::POSITION )
//       {
//         ROS_WARN_STREAM(ros::Time::now() << " fri_client.h - wrong command state in newJointPosCommand ");
//         return;
//       }
// 
//       if( new_joint_pos_command.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS )
//       {
//         throw std::runtime_error("Mismatch of New Command Vector"); 
//       }
// 
//       command_joint_position_.push_back( new_joint_pos_command );
// 
//       return;
//     }
    
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
      
//       std::cout<< "new commandded wrench: " << new_wrench_command[0] << "; " << new_wrench_command[1] << "; " << new_wrench_command[2] << "; " << std::endl;
    
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
    std::vector<double> joint_pos_command_;
    std::vector<double> initial_torque_;
    std::vector<double> initial_wrench_;
    KUKA::FRI::ESessionState actual_state_;
    KUKA::FRI::ESessionState actual_state_prev_;
    bool control_running_;
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> realtime_pub_;
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
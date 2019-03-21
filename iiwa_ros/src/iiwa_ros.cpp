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

#ifdef ENABLE_FRI
// #include <iiwa_msgs/ControlMode.h>
#endif
#include <eigen3/Eigen/Core>
#include <iiwa_msgs/ControlMode.h>
#include <iiwa_ros/iiwa_ros.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;

namespace iiwa_ros
{

ros::Time last_update_time;

iiwaRos::iiwaRos() { }

void iiwaRos::init(double fri_cycle_time, const bool verbosity)
{
  dt_ = fri_cycle_time;
    holder_state_pose_.init ( "state/CartesianPose" );
    holder_state_joint_position_.init ( "state/JointPosition" );
    holder_state_joint_torque_.init ( "state/JointTorque" );
    holder_state_wrench_.init ( "state/CartesianWrench" );
    holder_state_joint_stiffness_.init ( "state/JointStiffness" );
    holder_state_joint_position_velocity_.init ( "state/JointPositionVelocity" );
    holder_state_joint_damping_.init ( "state/JointDamping" );
    holder_state_joint_velocity_.init ( "state/JointVelocity" );
    holder_state_destination_reached_.init ( "state/DestinationReached" );

    holder_command_pose_.init ( "command/CartesianPose" );
    holder_command_payload_.init ( "command/Payload" );
    holder_command_joint_position_.init ( "command/JointPosition" );
    holder_command_joint_position_velocity_.init ( "command/JointPositionVelocity" );
    holder_command_joint_velocity_.init ( "command/JointVelocity" );

    servo_motion_service_.setServiceName ( "configuration/configureSmartServo" );
    path_parameters_service_.setServiceName ( "configuration/pathParameters" );
    time_to_destination_service_.setServiceName ( "state/timeToDestination" );
    
    servo_motion_service_.setVerbosity(verbosity);
    path_parameters_service_.setVerbosity(verbosity);
    time_to_destination_service_.setVerbosity(verbosity);
}

bool iiwaRos::getRobotIsConnected()
{
    ros::Duration diff = ( ros::Time::now() - last_update_time );
    return ( diff < ros::Duration ( 0.25 ) );
}
bool iiwaRos::isFRIModalityActive()
{
  int control_modality_active = servo_motion_service_.getControlModeActive();
  bool fri_modality_active    = ( control_modality_active == iiwa_msgs::ControlMode::FRI_JOINT_POS_CONTROL  )
                              || ( control_modality_active == iiwa_msgs::ControlMode::FRI_TORQUE_CONTROL    )
                              || ( control_modality_active == iiwa_msgs::ControlMode::FRI_JOINT_IMP_CONTROL );

  return fri_modality_active
      && servo_motion_service_.getFRIApp( ) != nullptr
      && servo_motion_service_.getFRIApp( )->isActive();

}

bool iiwaRos::getCartesianPose ( geometry_msgs::PoseStamped& value )
{
    if( !isFRIModalityActive() )
      return holder_state_pose_.get ( value );
    else
    {
      return servo_motion_service_.getFRIApp( )->getFRIClient().getCartesianPose(value);
    }
}

bool iiwaRos::getJointPosition ( iiwa_msgs::JointPosition& value )
{
  if( !isFRIModalityActive() )
    return holder_state_joint_position_.get ( value );
  else
  {
    return servo_motion_service_.getFRIApp( )->getFRIClient().getJointPosition(value);
  }
}

bool iiwaRos::getJointTorque ( iiwa_msgs::JointTorque& value )
{
  if( !isFRIModalityActive() )
    return holder_state_joint_torque_.get ( value );
  else
  {
    return servo_motion_service_.getFRIApp( )->getFRIClient().getJointTorque(value);
  }
}

bool iiwaRos::getJointStiffness ( iiwa_msgs::JointStiffness& value )
{
    return holder_state_joint_stiffness_.get ( value );
}

bool iiwaRos::getCartesianWrench (geometry_msgs::WrenchStamped& value , const char what, const bool compensate_payload)
{
  bool ret = true;
  
  if( !isFRIModalityActive() )
    ret = holder_state_wrench_.get ( value );
  else
  {
    ret =  servo_motion_service_.getFRIApp( )->getFRIClient().getCartesianWrench (value, what);
  }
  
  if(!ret)
    return false;
  
  if (compensate_payload && payload.initializated)
  {
    geometry_msgs::PoseStamped pose_msg;
    getCartesianPose(pose_msg);
    
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);
    
    Eigen::Matrix<double,6,1> wrench;
    tf::wrenchMsgToEigen(value.wrench,wrench);
    
    Eigen::Vector3d gravity_b; gravity_b << 0,0,-9.81;
    Eigen::Vector3d gravity_e = pose.linear().transpose() * gravity_b;
    
    Eigen::VectorXd wrench_ret(6);
    
    if(what == 'b')
    {
      wrench_ret.block(0,0,3,1) = wrench.block(0,0,3,1) + payload.mass * gravity_b;
      wrench_ret.block(3,0,3,1) = wrench.block(3,0,3,1) + pose.linear() * (payload.mass * payload.distance.cross(gravity_e));
    }
    else
    {
      wrench_ret.block(0,0,3,1) = wrench.block(0,0,3,1) + payload.mass * gravity_e;
      wrench_ret.block(3,0,3,1) = wrench.block(3,0,3,1) + payload.mass * payload.distance.cross(gravity_e);
    }
    
    tf::wrenchEigenToMsg(wrench_ret,value.wrench);
    
    return true;
  }
  
  
  return ret;
}

bool iiwaRos::getJacobian(Eigen::MatrixXd& value)
{
  if( !isFRIModalityActive() )
  {
    assert(0);
    return false;
  }
  else
  {
    return servo_motion_service_.getFRIApp( )->getFRIClient().getJacobian (value);
  }
}

bool iiwaRos::getJointVelocity ( iiwa_msgs::JointVelocity& value )
{
  if( !isFRIModalityActive() )
    return holder_state_joint_velocity_.get ( value );
  else
  {
    return servo_motion_service_.getFRIApp( )->getFRIClient().getJointVelocity (value);
  }
}

bool iiwaRos::getJointPositionVelocity ( iiwa_msgs::JointPositionVelocity& value )
{
    return holder_state_joint_position_velocity_.get ( value );
}

bool iiwaRos::getJointDamping ( iiwa_msgs::JointDamping& value )
{
    return holder_state_joint_damping_.get ( value );
}

void iiwaRos::setCartesianPose ( const geometry_msgs::PoseStamped& position )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_.getControlModeActive();
  if(( control_modality_active == 5 )
  || ( control_modality_active == 6 )
  || ( control_modality_active == 7 )
  || ( control_modality_active == 8 ) 
  )
  {
    throw std::runtime_error( "FRI : TODO" );
  }
  else
  {
    holder_command_pose_.set ( position );
    holder_command_pose_.publishIfNew();
  }
#else
    holder_command_pose_.set ( position );
    holder_command_pose_.publishIfNew();
#endif
}

void iiwaRos::setPayload ( const geometry_msgs::PoseStamped& position )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_.getControlModeActive();
  if(( control_modality_active == 5 )
  || ( control_modality_active == 6 )
  || ( control_modality_active == 7 )
  || ( control_modality_active == 8 ) 
  )
  {
    std::cout << "setting the new payload..." << std::endl;
    std::cout << position.pose << std::endl;
    holder_command_payload_.set ( position );
    holder_command_payload_.publishIfNew();
    ros::Duration(3.).sleep();
  }
  else
  {
    std::cout << "setting the new payload..." << std::endl;
    holder_command_payload_.set ( position );
    holder_command_payload_.publishIfNew();
    ros::Duration(3.).sleep();
  }
#else
    holder_command_payload_.set ( position );
    holder_command_payload_.publishIfNew();
#endif
}

void iiwaRos::setJointTorque ( const iiwa_msgs::JointTorque& torque )
{
#if defined( ENABLE_FRI )
  if( ( servo_motion_service_.getControlModeActive() == 6 ) )
  {
    servo_motion_service_.getFRIApp( )->getFRIClient().newJointTorqueCommand( torque );
  }
  else
  {
//     holder_command_joint_position_.set ( position );
//     holder_command_joint_position_.publishIfNew();
  }
#else
//   holder_command_joint_position_.set ( position );
//   holder_command_joint_position_.publishIfNew();
#endif
}

void iiwaRos::setWrench ( const iiwa_msgs::CartesianQuantity& wrench )
{
#if defined( ENABLE_FRI )
  if( ( servo_motion_service_.getControlModeActive() == 8 ) )
  {
    servo_motion_service_.getFRIApp( )->getFRIClient().newWrenchCommand( wrench );
  }
  else
  {
//     holder_command_joint_position_.set ( position );
//     holder_command_joint_position_.publishIfNew();
  }
#else
//   holder_command_joint_position_.set ( position );
//   holder_command_joint_position_.publishIfNew();
#endif
}

bool iiwaRos::setJointPosition ( const iiwa_msgs::JointPosition& position )
{

#if defined( ENABLE_FRI )
  if( isFRIModalityActive() )
  {
    return servo_motion_service_.getFRIApp( )->getFRIClient().newJointPosCommand( position );
  }
  else
  {
    holder_command_joint_position_.set ( position );
    holder_command_joint_position_.publishIfNew();
    return true;
  }
#else
  holder_command_joint_position_.set ( position );
  holder_command_joint_position_.publishIfNew();
#endif

}

void iiwaRos::setJointVelocity ( const iiwa_msgs::JointVelocity& velocity )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_.getControlModeActive();
  if( ( control_modality_active == 5 )
  ||  ( control_modality_active == 6 ) 
  ||  ( control_modality_active == 7 ) )
  {
    throw std::runtime_error( "FRI : TODO" );
  }
  else
  {
    holder_command_joint_velocity_.set ( velocity );
    holder_command_joint_velocity_.publishIfNew();
  }
#else
  holder_command_joint_velocity_.set ( velocity );
  holder_command_joint_velocity_.publishIfNew();
#endif

}

bool iiwaRos::estimatePayload(const double estimation_time, const double toll)
{
  ros::Time st = ros::Time::now();
  
  geometry_msgs::WrenchStamped wrench_msg;
  geometry_msgs::PoseStamped   pose_msg;
  Eigen::VectorXd              wrench(6); wrench.setZero();
  Eigen::Affine3d              pose;
  Eigen::Vector3d              torque; torque.setZero();
  
  
  if(!getCartesianPose(pose_msg))
  {
    ROS_ERROR("Problem in getCartesianPose");
    return false;
  }
  
  tf::poseMsgToEigen(pose_msg.pose,pose);
  
  int count = 0;
  ros::Rate r(50);
  
  while((ros::Time::now() - st).toSec() < estimation_time)
  {
    count ++;
    if(!getCartesianWrench(wrench_msg,'e'))
    {
      ROS_ERROR("Problem in getCartesianWrench");
      return false;
    }
    
     Eigen::Matrix<double,6,1> wrench_tmp;
    tf::wrenchMsgToEigen(wrench_msg.wrench,wrench_tmp);
    
    wrench += wrench_tmp;
    r.sleep();
  }
  
  wrench /= (double)count;
  payload.mass = wrench.block(0,0,3,1).norm()/9.81;
  
  Eigen::Vector3d gravity_b; gravity_b <<0, 0, -9.81;
  Eigen::Vector3d gravity_e = pose.linear().transpose() * gravity_b;
  
  torque = wrench.block(3,0,3,1);
  
  //Tx_e = m(-gz*dy + gy*dz)   --> dz = ( Tx_e/m + gz*dy)/gy
  //Ty_e = m(gz*dx - gx*dz )   --> dz = (-Ty_e/m + gz*dx)/gx 
  //Tz_e = m(-gy*dx + gx*dy)
  if(torque(2)/payload.mass > ( gravity_e(0)*toll + gravity_e(1)*toll))
    ROS_WARN("Warning: torque along z direction not negligible");
  
  if(gravity_e(0) < 0.01 && gravity_e(1) > 0.01)
    payload.distance <<0, 0,  (torque(0)/payload.mass)/gravity_e(1);
  else if(gravity_e(0) > 0.01 && gravity_e(1) < 0.01)
    payload.distance <<0, 0, -torque(1)/payload.mass/gravity_e(0);
  else if(gravity_e(0) < 0.01 && gravity_e(1) < 0.01)
  {
    ROS_ERROR("Impossible to compute the distance. return");
    return false;
  }
  else 
    payload.distance <<0, 0, 0.5 * ( (torque(0)/payload.mass)/gravity_e(1) + -torque(1)/payload.mass/gravity_e(0) );
  
  payload.initializated = true;
  return true;
}
  
void iiwaRos::setJointPositionVelocity ( const iiwa_msgs::JointPositionVelocity& value )
{
#if defined( ENABLE_FRI )
  int control_modality_active = servo_motion_service_.getControlModeActive();
  if( ( control_modality_active == 5 )
  ||  ( control_modality_active == 6 ) 
  ||  ( control_modality_active == 7 ) )
  {
    throw std::runtime_error( "FRI : TODO" );
  }
  else
  {
    holder_command_joint_position_velocity_.set ( value );
    holder_command_joint_position_velocity_.publishIfNew();
  }
#else
    holder_command_joint_position_velocity_.set ( value );
    holder_command_joint_position_velocity_.publishIfNew();
#endif
}

/*




    void iiwaState::iiwaStateJointPosition(const iiwa_msgs::JointPositionConstPtr& msg )
    {
      q_msg_ = *msg;

      if( first_q_ )
      {
        first_q_ = false;
        q_msg_.header.stamp = ros::Time::now();
        iiwa_ros::iiwaJointPositionToEigenVector(q_msg_, q_);
        dq_.setZero();
      }
      else
      {
        ros::Time last_time = q_msg_.header.stamp;
        q_msg_.header.stamp = ros::Time::now();
        double dt = ( q_msg_.header.stamp  -last_time ).toSec();
        Eigen::VectorXd q_prev = q_;
        iiwa_ros::iiwaJointPositionToEigenVector(q_msg_, q_);
        dq_ = ( q_ - q_prev ) / dt;
      }
      KDL::Frame cartpos;
      unsigned int nj = iiwa_tree_.getNrOfJoints();
      KDL::JntArray jointpositions = KDL::JntArray(nj);
      for(size_t i=0;i<nj;i++)
        jointpositions(i) = q_(i);
      bool kinematics_status = fksolver_->JntToCart(jointpositions,cartpos);
      tf::transformKDLToEigen(cartpos, pose_);
      KDL::Jacobian jac;
      jac.resize(iiwa_chain_.getNrOfJoints());
      int err = jacsolver_->JntToJac (jointpositions, jac );
      assert( jac.columns() == 7 );
      assert( jac.rows() == 6 );
      for(size_t i=0; i<6; i++)
        for(size_t j=0; j<7; j++)
          jacobian_(i,j) = jac(i,j);

      twist_    = jacobian_ * dq_;

      running_ = true;

    }
    void iiwaState::iiwaStateJointVelocity(const iiwa_msgs::JointVelocityConstPtr& msg )
    {
      dq_msg_ = *msg;
      dq_msg_.header.stamp = ros::Time::now();
      iiwa_ros::iiwaJointVelocityToEigenVector(dq_msg_, dq_);
    }
    void iiwaState::iiwaStateJointTorque(const iiwa_msgs::JointTorqueConstPtr& msg )
    {
      tau_msg_ = *msg;
      tau_msg_.header.stamp = ros::Time::now();
      iiwa_ros::iiwaJointTorqueToEigenVector(tau_msg_, tau_);
    }
    void iiwaState::iiwaStateWrench(const geometry_msgs::WrenchStampedConstPtr& msg )
    {

      wrench_ = *msg;
      wrench_.header.stamp = ros::Time::now();


      Eigen::Matrix<double,6,1> e;
      tf::wrenchMsgToEigen(msg->wrench, e);
      if( first_run_ )
      {

        f_ee_ << e(0),e(1),e(2);
        f0_   << e(0),e(1),e(2);
        tau_ee_ << e(3),e(4),e(5);
        tau0_   << e(3),e(4),e(5);

        first_run_ = false;
        std::cout<<"f0: "<< f_ee_.transpose()<<std::endl;
      }

      f_ee_ << e(0),e(1),e(2);
      f_ee_ -= f0_;
      f_b_   = pose_.linear() * f_ee_;

      tau_ee_ << e(3),e(4),e(5);
      tau_ee_ -= tau0_;
      tau_b_   = pose_.linear() * tau_ee_;

    }


    iiwaState::iiwaState ( const std::string& robot_description
              , const std::string &chain_root
              , const std::string &chain_tip
              , const Eigen::VectorXd& m, const Eigen::VectorXd& c, const Eigen::VectorXd& k )
    : running_   ( false )
    , q_         ( 7 )
    , dq_        ( 7 )
    , tau_       ( 7 )
    , jacobian_  ( 6, 7 )
    , first_run_ ( true )
    , first_q_   ( true )
    , M(6,6)
    , C(6,6)
    , K(6,6)
    {
      ros::NodeHandle nh("~");


      std::string robot_desc_string;
      nh.param(robot_description, robot_desc_string, std::string());
      if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree_))
      {
         ROS_ERROR("Failed to construct kdl tree");
         throw std::runtime_error("Failed to to construct kdl tree");
      }

      iiwa_tree_.getChain ( chain_root, chain_tip, iiwa_chain_);
      fksolver_.reset( new KDL::ChainFkSolverPos_recursive( iiwa_chain_ ) );
      jacsolver_.reset( new KDL::ChainJntToJacSolver( iiwa_chain_ ) );

      M  = m.asDiagonal();
      C  = c.asDiagonal();
      K  = k.asDiagonal();

      for(int i = 0; i<C.rows(); i++){
        for (int j = 0; j<C.cols(); j++){
          C(i,j) = 2*C(i,j)*sqrt(M(i,j)*K(i,j));
        }
      }

      ROS_INFO_STREAM( "Control parameters:\n matrix M\n"<< M << "\nmatrix C\n"<< C << "\nmatrix K\n"<< K  );

      q_       .setZero();
      dq_      .setZero();
      jacobian_.setZero();
      f_ee_    .setZero();
      f_b_     .setZero();
      f0_      .setZero();
      tau_ee_    .setZero();
      tau_b_     .setZero();
      tau0_      .setZero();


      sub_joint_position = nh.subscribe("/iiwa/state/JointPosition"  , 1000, &iiwaState::iiwaStateJointPosition  , this);
      sub_joint_velocity = nh.subscribe("/iiwa/state/JointVelocity"  , 1000, &iiwaState::iiwaStateJointVelocity  , this);
      sub_joint_torque   = nh.subscribe("/iiwa/state/JointTorque"    , 1000, &iiwaState::iiwaStateJointTorque    , this);
      sub_cart_wrench    = nh.subscribe("/iiwa/state/CartesianWrench", 1000, &iiwaState::iiwaStateWrench         , this);


    }

    iiwaState::~iiwaState( )
    {
    }


    bool iiwaState::getJointPosition(iiwa_msgs::JointPosition& jp)
    {
      jp = q_msg_;
      return CHECK_CLOCK( q_msg_.header.stamp, "GetJoitPosition" );
    }

    bool iiwaState::getJointPosition(Eigen::VectorXd& q)
    {

      q = q_;

      return CHECK_CLOCK( q_msg_.header.stamp, "GetJoitPosition" );
    }

    bool iiwaState::getCartesianPose( Eigen::Affine3d& cp)
    {

      cp = pose_;

      return CHECK_CLOCK( q_msg_.header.stamp, "getCartesianPose" );
    }

    bool iiwaState::getCartesianPoseVector(Eigen::VectorXd& x_msr)
    {
      geometry_msgs::PoseStamped pose;

      tf::poseEigenToMsg(pose_, pose.pose );

      x_msr = iiwa_ros::poseToVec( pose );
      return CHECK_CLOCK( q_msg_.header.stamp, "getCartesianPoseVector" );
    }

    bool iiwaState::getRotation( Eigen::Matrix3d& R  )
    {

      R = pose_.linear();

      return CHECK_CLOCK( q_msg_.header.stamp, "getRotation" );
    }

    bool iiwaState::getQuaternion( Eigen::Quaterniond& quat )
    {

      quat = Eigen::Quaterniond( pose_.linear() ).normalized();

      return CHECK_CLOCK( q_msg_.header.stamp, "getQuaternion" );
    }


    bool iiwaState::getCartesianPoint( Eigen::Vector3d& point )
    {
      Eigen::Affine3d cp;
      if(!getCartesianPose( cp ) ) return false;
      point(0) = cp.translation().x();
      point(1) = cp.translation().y();
      point(2) = cp.translation().z();
      return true;
    }

    bool iiwaState::getWrench( Eigen::VectorXd& ret, const char what)
    {

      Eigen::Vector3d f = (what == 'e' ) ? f_ee_  : f_b_;
      Eigen::Vector3d t = (what == 'e' ) ? tau_ee_  : tau_b_;
      ros::Time st = wrench_.header.stamp;

      ret.resize(6);
      ret << f(0),f(1),f(2),t(0),t(1),t(2);
      return CHECK_CLOCK( st, "getWrench" );
    }

    bool iiwaState::getForce( Eigen::Vector3d& ret, const char what)
    {
      Eigen::VectorXd tmp;
      if(!getWrench( tmp, what )) return false;
      ret << tmp(0), tmp(1), tmp(2);
      return true;
    }

    bool iiwaState::getTorque( Eigen::Vector3d& ret, const char what)
    {
      Eigen::VectorXd tmp;
      if(!getWrench( tmp, what )) return false;
      ret << tmp(3), tmp(4), tmp(5);
      return true;
    }

    bool iiwaState::getTwist( Eigen::VectorXd&  ret )
    {

      ret = twist_;

      return CHECK_CLOCK( q_msg_.header.stamp, "getTwist" );
    }

    bool iiwaState::getVelocity( Eigen::Vector3d& vel )
    {
      Eigen::VectorXd  vel6;
      if(!getTwist( vel6 )) return false;
      vel << vel6(0), vel6(1), vel6(2);
      return true;
    }

    bool iiwaState::getOmega( Eigen::Vector3d& omega )
    {
      Eigen::VectorXd  vel6;
      if(!getTwist( vel6 )) return false;
      omega << vel6(0), vel6(1), vel6(2);
      return true;
    }


    bool iiwaState::getMatrix( Eigen::Matrix3d& ret , const char what, const char translation )
    {
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++){
          ret(i,j)  = what ==  'M' ? M(i + ( translation ? 0 : 3  ) ,j + + ( translation ? 0 : 3  ) )
                    : what ==  'K' ? K(i + ( translation ? 0 : 3  ) ,j + + ( translation ? 0 : 3  ) )
                    : what ==  'C' ? C(i + ( translation ? 0 : 3  ) ,j + + ( translation ? 0 : 3  ) )
                    : std::nan("");
          assert( std::isfinite( ret(i,j) ) );
        }
      }
      return true;
    }

    bool iiwaState::toJointVelocity(const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega, Eigen::VectorXd& ret)
    {
      Eigen::VectorXd v(6);
      Eigen::Vector3d w_b;

      w_b = pose_.linear() * omega;
      for ( int idx=0; idx<3; idx++ )
      {
        v(idx) = velocity(idx);
        v(idx+3) = w_b(idx);
      }

      Eigen::JacobiSVD<Eigen::MatrixXd> svd_jac(jacobian_,  Eigen::ComputeThinU | Eigen::ComputeThinV);
      ret = svd_jac.solve(v);


      return true;
    }

    bool iiwaState::toJointVelocity(const Eigen::VectorXd& q, const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega, Eigen::VectorXd& ret)
    {

      unsigned int nj = iiwa_tree_.getNrOfJoints();
      KDL::JntArray jointpositions = KDL::JntArray(nj);
      for(size_t i=0;i<nj;i++)
        jointpositions(i) = q(i);

      KDL::Frame cartpos;
      Eigen::Affine3d pose;
      bool kinematics_status = fksolver_->JntToCart(jointpositions,cartpos);
      tf::transformKDLToEigen(cartpos, pose);

      KDL::Jacobian jac;
      jac.resize(iiwa_chain_.getNrOfJoints());
      jacsolver_->JntToJac (jointpositions, jac );
      Eigen::MatrixXd jacobian(6,7); jacobian.setZero();
      assert( jac.columns() == 7 );
      assert( jac.rows() == 6 );
      for(size_t i=0; i<6; i++)
        for(size_t j=0; j<7; j++)
          jacobian(i,j) = jac(i,j);

      Eigen::VectorXd v(6);
      Eigen::Vector3d w_b;

      w_b = pose.linear() * omega;
      for ( int idx=0; idx<3; idx++ )
      {
        v(idx) = velocity(idx);
        v(idx+3) = w_b(idx);
      }

      Eigen::JacobiSVD<Eigen::MatrixXd> svd_jac(jacobian,  Eigen::ComputeThinU | Eigen::ComputeThinV);
      ret = svd_jac.solve(v);


      return true;
    }


    bool iiwaState::isRunning() const { return running_; }

    */
}

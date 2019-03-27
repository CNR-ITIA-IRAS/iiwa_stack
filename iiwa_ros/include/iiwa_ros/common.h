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

#pragma once

#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/CartesianQuantity.h>

#if defined( ENABLE_FRI )
  #include <iiwa_fri/friLBRState.h>
#endif

#include <iiwa_msgs/CartesianVelocity.h>
#include <iiwa_msgs/CartesianQuantity.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointStiffness.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointDamping.h>

#include <std_msgs/Time.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_kdl.h>

namespace Eigen
{
  typedef Matrix<double, 7,  1> Vector7d;
  typedef Matrix<double, 7, -1> Matrix7Xd;
  typedef Matrix<double, 6,  1> Vector6d;
  typedef Matrix<double, 6, -1> Matrix6Xd;
  typedef Matrix<double, 6,  6> Matrix66d;
  typedef Matrix<double, 6,  7> Matrix67d;
}



namespace iiwa_ros 
{
  
  
  /**
   * @brief Creates a JointQuantity with the same value in all its components.
   * 
   * @param value the value to use for all the JointQuantity components.
   * @return iiwa_msgs::JointQuantity
   */
  inline iiwa_msgs::JointQuantity jointQuantityFromDouble(const double value) {
    iiwa_msgs::JointQuantity quantity;
    quantity.a1 = value;
    quantity.a2 = value;
    quantity.a3 = value;
    quantity.a4 = value;
    quantity.a5 = value;
    quantity.a6 = value;
    quantity.a7 = value;
    return quantity;
  }
  
  /**
   * @brief Creates a JointQuantity with the given values for as components.
   * 
   * @param a1
   * @param a2
   * @param a3
   * @param a4
   * @param a5
   * @param a6
   * @param a7
   * @return iiwa_msgs::JointQuantity
   */
  inline iiwa_msgs::JointQuantity jointQuantityFromDouble(const double a1, const double a2, const double a3, const double a4, const double a5, const double a6, const double a7) {
    iiwa_msgs::JointQuantity quantity;
    quantity.a1 = a1;
    quantity.a2 = a2;
    quantity.a3 = a3;
    quantity.a4 = a4;
    quantity.a5 = a5;
    quantity.a6 = a6;
    quantity.a7 = a7;
    return quantity;
  }
  
  /**
   * @brief Creates a CartesianQuantity with the same value in all its components.
   * 
   * @param value the value to use for all the CartesianQuantity components.
   * @return iiwa_msgs::CartesianQuantity
   */
  inline iiwa_msgs::CartesianQuantity CartesianQuantityFromDouble(const double value) {
    iiwa_msgs::CartesianQuantity quantity;
    quantity.x = value;
    quantity.y = value;
    quantity.z = value;
    quantity.a = value;
    quantity.b = value;
    quantity.c = value;
    return quantity;
  }
  
  /**
   * @brief Creates a CartesianQuantity with the given values for as components.
   * 
   * @param x 
   * @param y 
   * @param z 
   * @param a 
   * @param b 
   * @param c 
   * @return iiwa_msgs::CartesianQuantity
   */
  inline iiwa_msgs::CartesianQuantity CartesianQuantityFromDouble(const double x, const double y, const double z, const double a, const double b, const double c) {
    iiwa_msgs::CartesianQuantity quantity;
    quantity.x = x;
    quantity.y = y;
    quantity.z = z;
    quantity.a = a;
    quantity.b = b;
    quantity.c = c;
    return quantity;
  }
  
  /**
   * @brief Creates a CartesianQuantity with the given values for its translational and rotational component respectively.
   * 
   * @param translation_value value to use for all the transflational components (x,y,z) of the CartesianQuantity
   * @param rotation_value value to use for all the rotational components (a,b,c) of the CartesianQuantity
   * @return iiwa_msgs::CartesianQuantity
   */
  inline iiwa_msgs::CartesianQuantity CartesianQuantityFromDouble(const double translation_value, const double rotation_value) {
    iiwa_msgs::CartesianQuantity quantity;
    quantity.x = translation_value;
    quantity.y = translation_value;
    quantity.z = translation_value;
    quantity.a = rotation_value;
    quantity.b = rotation_value;
    quantity.c = rotation_value;
    return quantity;
  }
  
  inline std::vector<double> jointPositionToVector(const iiwa_msgs::JointQuantity& quantity) {
    std::vector<double> ret(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0);
    ret[0] = quantity.a1;
    ret[1] = quantity.a2;
    ret[2] = quantity.a3;
    ret[3] = quantity.a4;
    ret[4] = quantity.a5;
    ret[5] = quantity.a6;
    ret[6] = quantity.a7;
    return ret;
  }



  inline void eigenVectorToIiwaJointPosition( const Eigen::Vector7d& in, iiwa_msgs::JointPosition& out )
  {
      out.header.stamp = ros::Time::now();

      out.position.a1  = in(0);
      out.position.a2  = in(1);
      out.position.a3  = in(2);
      out.position.a4  = in(3);
      out.position.a5  = in(4);
      out.position.a6  = in(5);
      out.position.a7  = in(6);
  }
  inline void iiwaJointPositionToEigenVector( const iiwa_msgs::JointPosition& in, Eigen::Vector7d& out )
  {
      out.resize(7);
      out.setZero();
      out(0) = in.position.a1;
      out(1) = in.position.a2;
      out(2) = in.position.a3;
      out(3) = in.position.a4;
      out(4) = in.position.a5;
      out(5) = in.position.a6;
      out(6) = in.position.a7;
  }
  inline void iiwaJointVelocityToEigenVector( const iiwa_msgs::JointVelocity& in, Eigen::Vector7d& out )
  {
      out.resize(7);
      out.setZero();
      out(0) = in.velocity.a1;
      out(1) = in.velocity.a2;
      out(2) = in.velocity.a3;
      out(3) = in.velocity.a4;
      out(4) = in.velocity.a5;
      out(5) = in.velocity.a6;
      out(6) = in.velocity.a7;
  }

  inline void iiwaJointTorqueToEigenVector( const iiwa_msgs::JointTorque& in, Eigen::Vector7d& out )
  {
      out.resize(7);
      out.setZero();
      out(0) = in.torque.a1;
      out(1) = in.torque.a2;
      out(2) = in.torque.a3;
      out(3) = in.torque.a4;
      out(4) = in.torque.a5;
      out(5) = in.torque.a6;
      out(6) = in.torque.a7;
  }

  inline geometry_msgs::WrenchStamped toWrenchStamped(const Eigen::Vector6d& wrench)
  {
      geometry_msgs::WrenchStamped msg;

      msg.header.stamp = ros::Time::now();

      msg.wrench.force.x = wrench(0);
      msg.wrench.force.y = wrench(1);
      msg.wrench.force.z = wrench(2);

      msg.wrench.torque.x = wrench(3);
      msg.wrench.torque.y = wrench(4);
      msg.wrench.torque.z = wrench(5);

      return msg;
  }

  inline geometry_msgs::TwistStamped toTwistStamped(const Eigen::Vector6d& velocity)
  {
      geometry_msgs::TwistStamped msg;

      msg.header.stamp = ros::Time::now();

      msg.twist.linear.x = velocity(0);
      msg.twist.linear.y = velocity(1);
      msg.twist.linear.z = velocity(2);

      msg.twist.angular.x = velocity(3);
      msg.twist.angular.y = velocity(4);
      msg.twist.angular.z = velocity(5);

      return msg;
  }

  inline geometry_msgs::TwistStamped toPoseStamped(const Eigen::Vector3d& t,const Eigen::Vector3d& r)
  {
      geometry_msgs::TwistStamped msg;

      msg.header.stamp = ros::Time::now();

      msg.twist.linear.x = t(0);
      msg.twist.linear.y = t(1);
      msg.twist.linear.z = t(2);

      msg.twist.angular.x = r(0);
      msg.twist.angular.y = r(1);
      msg.twist.angular.z = r(2);

      return msg;
  }

  inline Eigen::Vector6d poseToVec(const geometry_msgs::PoseStamped p)
  {
    Eigen::VectorXd v(6);

    v(0) = p.pose.position.x;
    v(1) = p.pose.position.y;
    v(2) = p.pose.position.z;

    double sinr = +2.0 * (p.pose.orientation.w * p.pose.orientation.x + p.pose.orientation.y * p.pose.orientation.z);
    double cosr = +1.0 - 2.0 * (p.pose.orientation.x * p.pose.orientation.x + p.pose.orientation.y * p.pose.orientation.y);
    v(3) = atan2(sinr, cosr);

    double sinp = +2.0 * (p.pose.orientation.w * p.pose.orientation.y - p.pose.orientation.z * p.pose.orientation.x);
    if (fabs(sinp) >= 1)
      v(4) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
      v(4) = asin(sinp);

    double siny = +2.0 * (p.pose.orientation.w * p.pose.orientation.z + p.pose.orientation.x * p.pose.orientation.y);
    double cosy = +1.0 - 2.0 * (p.pose.orientation.y * p.pose.orientation.y + p.pose.orientation.z * p.pose.orientation.z);
    v(5) = atan2(siny, cosy);
    return v;
  }

}

/*
 * CBfrDriver.cpp
 *
 *  Created on: Apr 28, 2013
 *      Author: Samir Menon
 *       Email: <smenon@stanford.edu>
 */

#include "CBfrDriver.hpp"

#include <iostream>
#include <cmath>
#include <cassert>
#include <stdio.h>

namespace bfr
{

  CBfrDriver::CBfrDriver() :
        has_been_init_(false),
        q0_(0), q1_(0), q2_(0),
        dq0_(0), dq1_(0), dq2_(0),
        ddq0_(0), ddq1_(0), ddq2_(0),
        q0_raw_(0), q1_raw_(0), q2_raw_(0),
        q0_raw_init_(0), q1_raw_init_(0), q2_raw_init_(0),
        fq0_(0), fq1_(0), fq2_(0),
        x_ee_(0), y_ee_(0), z_ee_(0),
        dx_ee_(0), dy_ee_(0), dz_ee_(0),
        fx_ee_(0), fy_ee_(0), fz_ee_(0),
        fq0_grav_(0), fq1_grav_(0), fq2_grav_(0),
        flag_grav_compensation_enabled_(false),
        servo_ticks_(0)
  { time_[0] = 0; time_[1] = 0; time_[2] = 0; }

  bool CBfrDriver::init()
  {
    bool flag;
    flag = sensoray::CSensoray3DofIODriver::init();
    if(false == flag)
    { std::cout<<"\nCBfrDriver::init() : Could not initialize driver.";  }

    //Read the first encoder values.
    flag = sensoray::CSensoray3DofIODriver::readEncoders(q0_raw_init_, q1_raw_init_, q2_raw_init_);
    if(false == flag)
    { std::cout<<"\nCBfrDriver::init() : Could not read initial encoder position.";  }

    // Random other checks
    assert(max_amps_>0);

    has_been_init_ = flag;

    return flag;
  }

  void CBfrDriver::shutdown()
  {
    if(has_been_init_)
    { sensoray::CSensoray3DofIODriver::shutdown();  }
  }


  bool CBfrDriver::readGCAngles(double& arg_q0, double& arg_q1, double& arg_q2)
  {
    // Should only be called after initialization
    if(!has_been_init_) { return has_been_init_;  }

    bool flag;
    // Talk to the main board
    flag = sensoray::CSensoray3DofIODriver::readEncoders(q0_raw_, q1_raw_, q2_raw_);

    if(true == flag)
    {//If transaction was successful, update the joint angles
      q0_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear0_);
      q1_ = 2 * 3.1416 * (q1_raw_ - q1_raw_init_) / (encoder_counts_per_rev_ * gear1_);
      q2_ = 2 * 3.1416 * (q2_raw_ - q2_raw_init_) / (encoder_counts_per_rev_ * gear2_);

#ifdef DEBUG
      std::cout<<"\nEnc raw : "<<q0_raw_<<", "<<q1_raw_<<", "<<q2_raw_;
      std::cout<<"\nEnc init: "<<q0_raw_init_<<", "<<q1_raw_init_<<", "<<q2_raw_init_;
      std::cout<<"\nJoint   : "<<q0_<<", "<<q1_<<", "<<q2_;
#endif
    }

    arg_q0 = q0_;
    arg_q1 = q1_;
    arg_q2 = q2_;

    servo_ticks_++;
    return flag;
  }

  bool CBfrDriver::readGCAnglesAndCommandGCForces(double& arg_q0, double& arg_q1, double& arg_q2,
      const double arg_fq0, const double arg_fq1, const double arg_fq2)
  {
    // Should only be called after initialization
    if(!has_been_init_) { return has_been_init_;  }

    bool flag = true, flag2 = true;

    // Add gravity gc torques if enabled
    if(flag_grav_compensation_enabled_)
    {
      // Gravity compensation. Add gravity gc torques.
      flag = computeCurrGcGravity();
      if(false == flag) { return false; }
      fq0_ = arg_fq0 + fq0_grav_;
      fq1_ = arg_fq1 + fq1_grav_;
      fq2_ = arg_fq2 + fq2_grav_;
    }
    else
    {
      // No gravity compensation. Just use normal gc torques.
      fq0_ = arg_fq0;
      fq1_ = arg_fq1;
      fq2_ = arg_fq2;
    }

    // Transform the gc torques into driver inputs
    double i0,i1,i2;
    i0 = motor0_polarity_ * fq0_ / (gear0_* maxon_tau_per_amp_ * i_to_a0_);
    i1 = motor1_polarity_ * fq1_ / (gear1_* maxon_tau_per_amp_ * i_to_a1_);
    i2 = motor2_polarity_ * fq2_ / (gear2_* maxon_tau_per_amp_ * i_to_a2_);

    // If one or more motors exceeded the max current, limit it to the max.
    if(fabs(i0) > max_amps_/i_to_a0_) { flag = false; i0>=0? i0 = max_amps_/i_to_a0_: i0 = -1*max_amps_/i_to_a0_; }
    if(fabs(i1) > max_amps_/i_to_a1_) { flag = false; i1>=0? i1 = max_amps_/i_to_a1_: i1 = -1*max_amps_/i_to_a1_; }
    if(fabs(i2) > max_amps_/i_to_a2_) { flag = false; i2>=0? i2 = max_amps_/i_to_a2_: i2 = -1*max_amps_/i_to_a2_; }

    // Talk to the main board
    flag2 = sensoray::CSensoray3DofIODriver::readEncodersAndCommandMotors(q0_raw_, q1_raw_, q2_raw_, i0, i1, i2);
#ifdef DEBUG
    std::cout<<"\nSending driver inputs = "<<i0<<", "<<i1<<", "<<i2<<std::flush;
#endif

    if(true == flag2)
    {//If transaction was successful, update the joint angles
      q0_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear0_);
      q1_ = 2 * 3.1416 * (q1_raw_ - q1_raw_init_) / (encoder_counts_per_rev_ * gear1_);
      q2_ = 2 * 3.1416 * (q2_raw_ - q2_raw_init_) / (encoder_counts_per_rev_ * gear2_);
    }

    arg_q0 = q0_;
    arg_q1 = q1_;
    arg_q2 = q2_;

    servo_ticks_++;
    // Return success only if motor inputs were within limits AND the sensoray transaction was successful.
    return (flag && flag2);
  }

  bool CBfrDriver::readEEPosition(double& arg_x, double& arg_y, double& arg_z)
  {
    // Should only be called after initialization
    if(!has_been_init_) { return has_been_init_;  }

    bool flag; double tmp0, tmp1, tmp2;
    // Run the servo loop.
    flag = readGCAngles(tmp0, tmp1, tmp2);

    // Do the math to get the end-effector position
    flag = flag && computeCurrEEPosition();

    // Return the end effector position
    getEEPosition(arg_x, arg_y, arg_z);

    servo_ticks_++;
    return flag;
  }

  bool CBfrDriver::readEEPositionAndCommandEEForce(double& arg_x, double& arg_y, double& arg_z,
          const double arg_fx, const double arg_fy, const double arg_fz)
  {
    // Should only be called after initialization
    if(!has_been_init_) { return has_been_init_;  }

    bool flag = true;
    //Compute the current Jacobian
    flag = flag && computeCurrEEJacobian();

    //Compute the forces to be applied
    Eigen::Vector3d f_ee, f_q;
    f_ee<<arg_fx, arg_fy, arg_fz;
    f_q = J_ee_.transpose() * f_ee;

    //Now run the servo
    double tmp0, tmp1, tmp2, fq0, fq1, fq2;
    fq0 =f_q(0); fq1 = f_q(1); fq2 = f_q(2);

    // Run the servo loop.
    flag = flag && readGCAnglesAndCommandGCForces(tmp0, tmp1, tmp2, fq0, fq1, fq2);

    // Do the math to get the end-effector position
    flag = flag && computeCurrEEPosition();

    // Return the end effector position
    getEEPosition(arg_x, arg_y, arg_z);

    return flag;
  }

  bool CBfrDriver::computeCurrEEPosition()
  {
    // Should only be called after initialization
    if(!has_been_init_) { return has_been_init_;  }

    //Compute the forward kinematics
    x_ee_ = D3 * cos(q0_) - (D1 + D2) * sin(q0_) + cos(q0_) * ( (L1 + L5) * cos(q1_) -L4 * sin(q2_) );
    y_ee_ = L4 * cos(q2_) + (L1+L5) * sin(q1_);
    z_ee_ = (D1+D2) * cos(q0_) + D3 * sin(q0_) + sin(q0_) * ( (L1+L5) *cos(q1_) - L4 * sin(q2_) );

#ifdef DEBUG
    std::cout<<"\n End-effector position = "<<x_ee_<<", "<<y_ee_<<", "<<z_ee_;
#endif

    return true;
  }

  bool CBfrDriver::computeCurrEEJacobian()
  {
    // Should only be called after initialization
    if(!has_been_init_) { return has_been_init_;  }
    J_ee_(0,0) = (-1)*(D1+D2)*cos(q0_)+(-1)*D3*sin(q0_)+(-1)*sin(q0_)*((L1+L5)*cos(q1_)+(-1)*L4*sin(q2_));
    J_ee_(0,1) = (-1)*(L1+L5)*cos(q0_)*sin(q1_);
    J_ee_(0,2) = (-1)*L4*cos(q0_) *cos(q2_);
    J_ee_(1,0) = 0.0;
    J_ee_(1,1) = (L1+L5)*cos(q1_);
    J_ee_(1,2) = (-1)*L4*sin(q2_);
    J_ee_(2,0) = D3*cos(q0_)+(-1)*(D1+D2)*sin(q0_)+cos(q0_)*((L1+L5)*cos(q1_)+(-1)*L4*sin(q2_));
    J_ee_(2,1) = (-1)*(L1+L5)*sin(q0_)*sin(q1_);
    J_ee_(2,2) = (-1)*L4*cos(q2_)*sin(q0_);

#ifdef DEBUG
    std::cout<<"\n End-effector Jacobian = \n"<<J_ee_;
#endif

    return true;
  }

  bool CBfrDriver::computeCurrGcGravity()
  {
    // Should only be called after initialization
    if(!has_been_init_) { return has_been_init_;  }
    if(!flag_grav_compensation_enabled_) { return flag_grav_compensation_enabled_;  }

    // Gravity Compensation
    fq0_grav_ = -g*m1*( cos(q0_)*(L4*sin(q2_)-cos(q1_)*(L1*(1.0/2.0)+L5*(1.0/2.0))) + sin(q0_)*(D1+D2)-D3*cos(q0_))
                +g*m2*(-sin(q0_)*(D1+D2)+D3*cos(q0_)+L6*cos(q0_)*cos(q1_))
                -g*m0*(D4*cos(q0_)+D5*sin(q0_))
                -g*m4*(sin(q0_)*(D1+D2)-D3*cos(q0_)+L7*cos(q0_)*sin(q2_))
                +g*m3*(-sin(q0_)*(D1+D2)+D3*cos(q0_)+cos(q0_)*(L2*cos(q1_)-L3*sin(q2_)*(1.0/2.0)));
    fq0_grav_ = -fq0_grav_/30;

    fq1_grav_ = -L2*g*m3*sin(q0_)*sin(q1_)-L6*g*m2*sin(q0_)*sin(q1_)-g*m1*sin(q0_)*sin(q1_)*(L1*(1.0/2.0)+L5*(1.0/2.0));
    fq1_grav_ = fq1_grav_/20;

    fq2_grav_ = -L4*g*m1*cos(q2_)*sin(q0_)-L3*g*m3*cos(q2_)*sin(q0_)*(1.0/2.0)-L7*g*m4*cos(q2_)*sin(q0_);
    fq2_grav_ = fq2_grav_/20;

#ifdef DEBUG
    std::cout<<"\n End-effector gravity = "<<fq0_grav_<<", "<<fq1_grav_<<", "<<fq2_grav_;
#endif

    return true;
  }
} /* namespace bfr */

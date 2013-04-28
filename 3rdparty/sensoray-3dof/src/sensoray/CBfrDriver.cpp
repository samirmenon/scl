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
          q0_(0), q1_(0), q2_(0),
          dq0_(0), dq1_(0), dq2_(0),
          ddq0_(0), ddq1_(0), ddq2_(0),
          q0_raw_(0), q1_raw_(0), q2_raw_(0),
          q0_raw_init_(0), q1_raw_init_(0), q2_raw_init_(0),
          x_ee_(0), y_ee_(0), z_ee_(0),
          dx_ee_(0), dy_ee_(0), dz_ee_(0),
          fx_ee_(0), fy_ee_(0), fz_ee_(0)
  {}

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

    return flag;
  }

  void CBfrDriver::shutdown()
  { sensoray::CSensoray3DofIODriver::shutdown();  }


  bool CBfrDriver::readGcAngles(double& arg_q0, double& arg_q1, double& arg_q2)
  {
    bool flag;
    // Talk to the main board
    flag = sensoray::CSensoray3DofIODriver::readEncoders(q0_raw_, q1_raw_, q2_raw_);

    if(true == flag)
    {//If transaction was successful, update the joint angles
      q0_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear0_);
      q1_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear1_);
      q2_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear2_);
    }

    arg_q0 = q0_;
    arg_q1 = q1_;
    arg_q2 = q2_;

    return flag;
  }

  bool CBfrDriver::readGcAnglesAndCommandGcTorques(double& arg_q0, double& arg_q1, double& arg_q2,
      const double m0, const double m1, const double m2)
  {
    bool flag = true, flag2 = true;
    // Transform the gc torques into driver inputs
    double i0,i1,i2;
    i0 = m0 / (gear0_* maxon_tau_per_amp_ * i_to_a0_);
    i1 = m1 / (gear1_* maxon_tau_per_amp_ * i_to_a1_);
    i2 = m2 / (gear2_* maxon_tau_per_amp_ * i_to_a2_);

    // If one or more motors exceeded the max current, limit it to the max.
    if(fabs(i0) > max_amps_/i_to_a0_) { flag = false; i0>=0? i0 = max_amps_/i_to_a0_: i0 = -1*max_amps_/i_to_a0_; }
    if(fabs(i1) > max_amps_/i_to_a1_) { flag = false; i1>=0? i1 = max_amps_/i_to_a1_: i1 = -1*max_amps_/i_to_a1_; }
    if(fabs(i2) > max_amps_/i_to_a2_) { flag = false; i2>=0? i2 = max_amps_/i_to_a2_: i2 = -1*max_amps_/i_to_a2_; }

    // Talk to the main board
    flag2 = sensoray::CSensoray3DofIODriver::readEncodersAndCommandMotors(q0_raw_, q1_raw_, q2_raw_, i0, i1, i2);

    if(true == flag2)
    {//If transaction was successful, update the joint angles
      q0_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear0_);
      q1_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear1_);
      q2_ = 2 * 3.1416 * (q0_raw_ - q0_raw_init_) / (encoder_counts_per_rev_ * gear2_);
    }

    arg_q0 = q0_;
    arg_q1 = q1_;
    arg_q2 = q2_;

    // Return success only if motor inputs were within limits AND the sensoray transaction was successful.
    return (flag && flag2);
  }

} /* namespace bfr */

/*
 * CBfrDriver.hpp
 *
 *  Created on: Apr 28, 2013
 *      Author: Samir Menon
 *       Email: <smenon@stanford.edu>
 */

#ifndef CBFRDRIVER_HPP_
#define CBFRDRIVER_HPP_

#include "CSensoray3DofIODriver.hpp"
#include "SBfrKinematicAndInertialParams.hpp"

#include <Eigen/Core>

namespace bfr
{

  class CBfrDriver : private sensoray::CSensoray3DofIODriver, private bfr::SBfrKinematicAndInertialParamsV2
  {
  public:
    // *****************************************************
    // *********              Init                **********
    // *****************************************************
    CBfrDriver();
    virtual ~CBfrDriver() {}

    /** Initializes the Bfr driver */
    bool init();

    /** Closes the driver and shuts down the modules */
    void shutdown();

    // *****************************************************
    // *********         Communication            **********
    // ********* USE CAREFULLY: INVOLVE H/W Commn **********
    // *****************************************************
    /** Position operation only : Reads encoders */
    bool readGCAngles(double& arg_q0, double& arg_q1, double& arg_q2);

    /** Position+Force operation : Sends analog out to motors + reads encoders
     * Inputs are gc torques in Nm
     *
     * NOTE : If you aren't familiar with torques.
     *        Very High torque = 5 Nm
     *        High torque =  3 Nm
     *        Med torque = 1.5 Nm
     *        Low torque = 0.6 Nm
     *        jr3 noise = ~0.1 Nm
     */
    bool readGCAnglesAndCommandGCForces(double& arg_q0, double& arg_q1, double& arg_q2,
        const double arg_fq0, const double arg_fq1, const double arg_fq2);

    /** Position only operation : Sends analog out to motors + reads encoders
     * Inputs are end-effector forces in N. */
    bool readEEPosition(double& arg_x, double& arg_y, double& arg_z);

    /** Encoder+Motor operation : Sends analog out to motors + reads encoders
     * Inputs are end-effector forces in N.
     *
     * NOTE : If you aren't familiar with forces.
     *        High torque = 3.5 N
     *        Med torque = 1.5 N
     *        Low torque = 0.5 N
     *        jr3 noise = ~0.2 N */
    bool readEEPositionAndCommandEEForce(double& arg_x, double& arg_y, double& arg_z,
        const double arg_fx, const double arg_fy, const double arg_fz);

    // *****************************************************
    // *********             State                **********
    // *****************************************************
    void gravityCompensationOn(bool arg_flag)
    {  flag_grav_compensation_enabled_ = arg_flag;  }

    bool modeGravityCompensated()
    { return flag_grav_compensation_enabled_; }

    bool modePositionOnly()
    { return sensoray::CSensoray3DofIODriver::modeEncoderOnly();  }

    bool modePositionAndForce()
    { return sensoray::CSensoray3DofIODriver::modeEncoderAndMotor();  }

    // *****************************************************
    // *********          State I/O               **********
    // *****************************************************
    void getGCPosition(double& arg_q0, double& arg_q1, double& arg_q2)
    { arg_q0 = q0_; arg_q1 = q1_; arg_q2 = q2_;  }

    void getGCVelocity(double& arg_dq0, double& arg_dq1, double& arg_dq2)
    { arg_dq0 = dq0_; arg_dq1 = dq1_; arg_dq2 = dq2_;  }

    void getGCForce(double& arg_fq0, double& arg_fq1, double& arg_fq2)
    { arg_fq0 = fq0_; arg_fq1 = fq1_; arg_fq2 = fq2_;  }

    void getEEPosition(double& arg_x, double& arg_y, double& arg_z)
    { arg_x = x_ee_; arg_y = y_ee_; arg_z = z_ee_;  }

    void getEEZeroPosition(double& arg_x, double& arg_y, double& arg_z)
    { arg_x = x_ee_zero_; arg_y = y_ee_zero_; arg_z = z_ee_zero_;  }

    void getEEForce(double& arg_fx, double& arg_fy, double& arg_fz)
    { arg_fx = fx_ee_; arg_fy = fy_ee_; arg_fz = fz_ee_;  }

    long long getServoTicks()
    { return servo_ticks_;  }

  private:
    // *****************************************************
    // *********         Kinematics               **********
    // *****************************************************
    /** Compute and save the end-effector position using the current
     * generalized coordinates */
    bool computeCurrEEPosition();

    /** Compute and save the end-effector Jacobian using the current
     * generalized coordinates */
    bool computeCurrEEJacobian();

    /** Compute and save the end-effector Jacobian using the current
     * generalized coordinates */
    bool computeCurrGcGravity();

    /** Compute and save the end-effector Jacobian using the current
     * generalized coordinates */
    bool computeGcDynamics()
    { return false; }

  private:
    bool has_been_init_;
    // *****************************************************
    // *********        Position Params           **********
    // *****************************************************
    double q0_, q1_, q2_;
    double dq0_, dq1_, dq2_;
    double ddq0_, ddq1_, ddq2_;
    long q0_raw_, q1_raw_, q2_raw_;
    long q0_raw_init_, q1_raw_init_, q2_raw_init_;
    double fq0_, fq1_, fq2_;

    double x_ee_, y_ee_, z_ee_;
    double x_ee_zero_, y_ee_zero_, z_ee_zero_;
    double dx_ee_, dy_ee_, dz_ee_;
    double fx_ee_, fy_ee_, fz_ee_;

    // *****************************************************
    // *********        Dynamics Params           **********
    // *****************************************************
    double time_[3]; // Save the time for the last three servo loops.
    double fq0_grav_, fq1_grav_, fq2_grav_;
    Eigen::Matrix3d J_ee_;

    // *****************************************************
    // *********          Flag Params             **********
    // *****************************************************
    bool flag_grav_compensation_enabled_;
    long long servo_ticks_;

    // *****************************************************
    // *********       Calibration Params         **********
    // *****************************************************
    static const double max_amps_ = 2.5;

    //Amp output = i_to_a * i; //Amps per unit input
    static const double i_to_a0_ = 1.9846; //Amps / unit-input
    static const double i_to_a1_ = 1.9863;
    static const double i_to_a2_ = 2.0338;

    // Motor related params
    static const double maxon_tau_per_amp_ = .0578;// = 57.8 / 1000
    static const double motor0_polarity_ = -1.0;
    static const double motor1_polarity_ = -1.0;
    static const double motor2_polarity_ = 1.0;

    // Gearbox (capstan / shaft)
    static const double gear0_ = 30.0;
    static const double gear1_ = 20.0;
    static const double gear2_ = 20.0;

    // Encoder params
    static const double encoder_counts_per_rev_ = 10000.0; //Includes quadrature (2500 * 4)

    // Regression Fits
  };

} /* namespace bfr */
#endif /* CBFRDRIVER_HPP_ */

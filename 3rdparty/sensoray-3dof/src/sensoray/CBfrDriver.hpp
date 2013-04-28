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

namespace bfr
{

  class CBfrDriver : private sensoray::CSensoray3DofIODriver, private bfr::SBfrKinematicAndInertialParamsV2
  {
  public:
    CBfrDriver();
    virtual ~CBfrDriver() {}

    /** Initializes the Bfr driver */
    bool init();

    /** Closes the driver and shuts down the modules */
    void shutdown();

    /** Position operation only : Reads encoders */
    bool readGcAngles(double& arg_q0, double& arg_q1, double& arg_q2);

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
    bool readGcAnglesAndCommandGcTorques(double& arg_q0, double& arg_q1, double& arg_q2,
        const double arg_fq0, const double arg_fq1, const double arg_fq2);

    /** Position only operation : Sends analog out to motors + reads encoders
     * Inputs are end-effector forces in N. */
    bool readEEPosition(double& arg_x, double& arg_y, double& arg_z)
    { return false; }

    /** Encoder+Motor operation : Sends analog out to motors + reads encoders
     * Inputs are end-effector forces in N. */
    bool readEEPositionAndCommandEEForce(double& arg_x, double& arg_y, double& arg_z,
        const double arg_fx, const double arg_fy, const double arg_fz)
    { return false; }

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

  private:
    // *****************************************************
    // *********         Kinematics               **********
    // *****************************************************
    /** Compute and save the end-effector position using the current
     * generalized coordinates */
    bool computeCurrEEPosition()
    { return false; }

    /** Compute and save the end-effector Jacobian using the current
     * generalized coordinates */
    bool computeCurrEEJacobian()
    { return false; }

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

    double x_ee_, y_ee_, z_ee_;
    double dx_ee_, dy_ee_, dz_ee_;
    double fx_ee_, fy_ee_, fz_ee_;

    // *****************************************************
    // *********        Dynamics Params           **********
    // *****************************************************
    double fq0_grav_, fq1_grav_, fq2_grav_;

    // *****************************************************
    // *********          Flag Params             **********
    // *****************************************************
    bool flag_grav_compensation_enabled_;

    // *****************************************************
    // *********       Calibration Params         **********
    // *****************************************************
    static const double max_amps_ = 2.5;

    //Amp output = i_to_a * i; //Amps per unit input
    static const double i_to_a0_ = 1.9846; //Amps / unit-input
    static const double i_to_a1_ = 1.9863;
    static const double i_to_a2_ = 2.0338;

    static const double maxon_tau_per_amp_ = .0578;// = 57.8 / 1000

    static const double gear0_ = 30.0;
    static const double gear1_ = 20.0;
    static const double gear2_ = 20.0;

    static const double encoder_counts_per_rev_ = 10000.0; //Includes quadrature (2500 * 4)
  };

} /* namespace bfr */
#endif /* CBFRDRIVER_HPP_ */

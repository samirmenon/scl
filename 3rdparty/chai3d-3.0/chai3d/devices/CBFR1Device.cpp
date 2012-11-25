//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

 * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

 * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Your Name!
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 799 $
 */
//===========================================================================


//---------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CBFR1Device.h"
//---------------------------------------------------------------------------
#if defined(C_ENABLE_BFR_DEVICE_SUPPORT)
//---------------------------------------------------------------------------

/************************************************************************
    DOCUMENTATION:

    Please check header file CBFR1Device.h for some initial
    guidelines about how to implement your own haptic device using this
    following template.

    When ready, simply follow the next 12 documented steps.
 ************************************************************************/

const double HD_ENCODER_RESOLUTION_0 = 10000;
const double HD_ENCODER_RESOLUTION_1 = 10000;
const double HD_ENCODER_RESOLUTION_2 = 10000;

const double HD_GEAR_RATIO_0 = 30.0;
const double HD_GEAR_RATIO_1 = 20.0;
const double HD_GEAR_RATIO_2 = 20.0;

const double HD_LENGTH_0 = 0.1524;
const double HD_LENGTH_1 = 0.4953;
const double HD_LENGTH_2 = 1.2446;


//===========================================================================
/*!
    Constructor of cBFR1Device.

    \fn     cBFR1Device::cBFR1Device(unsigned int a_deviceNumber)
 */
//===========================================================================
cBFR1Device::cBFR1Device(unsigned int a_deviceNumber)
{
  // the connection to your device has not yet been established.
  m_deviceReady = false;


  /************************************************************************
        STEP 1:
        Here you should complete the specifications of your device.
        These values only need to be estimates. Since haptic devices often perform
        differently depending of their configuration withing their workspace,
        simply use average values.
   *************************************************************************/

  //-----------------------------------------------------------------------
  // NAME:
  //-----------------------------------------------------------------------

  // haptic device model (see file "CGenericHapticDevice.h")
  m_specifications.m_model                         = C_HAPTIC_DEVICE_BFR;

  // name of the device manufacturer, research lab, university.
  m_specifications.m_manufacturerName              = "Manips Lab. Stanford University.";

  // name of your device
  m_specifications.m_modelName                     = "BFR 1";


  //-----------------------------------------------------------------------
  // CHARACTERISTICS: (The following values must be positif or equal to zero)
  //-----------------------------------------------------------------------

  // the maximum force [N] the device can produce along the x,y,z axis.
  m_specifications.m_maxLinearForce                = 10.0; // [N]

  // the maximum amount of torque your device can provide arround its
  // rotation degrees of freedom.
  m_specifications.m_maxAngularTorque              = 0.0;  // [N*m]


  // the maximum amount of torque which can be provided by your gripper
  m_specifications.m_maxGripperForce               = 0.0;  // [N]

  // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
  m_specifications.m_maxLinearStiffness            = 100.0; // [N/m]

  // the maximum amount of angular stiffness
  m_specifications.m_maxAngularStiffness           = 1.0;  // [N*m/Rad]

  // the maximum amount of stiffness supported by the gripper
  m_specifications.m_maxGripperLinearStiffness      = 1.0;  // [N*m]

  // the radius of the physical workspace of the device (x,y,z axis)
  m_specifications.m_workspaceRadius               = 0.4; // [m]

  // DAMPING PROPERTIES:
  // Start with small values as damping terms can be high;y sensitive to
  // the quality of your velocity signal and the spatial resolution of your
  // device. Try gradually increasing the values by using example "01-devices" and by
  // enabling viscosity with key command "2".

  // Maximum recommended linear damping factor Kv
  m_specifications.m_maxLinearDamping           = 20.0;     // [N/(m/s)]

  //! Maximum recommended angular damping factor Kv (if actuated torques are available)
  m_specifications.m_maxAngularDamping        = 0.0;    // [N*m/(Rad/s)]

  //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
  m_specifications.m_maxGripperAngularDamping     = 0.0; // [N*m/(Rad/s)]


  //-----------------------------------------------------------------------
  // CHARACTERISTICS: (The following are of boolean type: (true or false)
  //-----------------------------------------------------------------------

  // does your device provide sensed position (x,y,z axis)?
  m_specifications.m_sensedPosition                = true;

  // does your device provide sensed rotations (i.e stylus)?
  m_specifications.m_sensedRotation                = false;

  // does your device provide a gripper which can be sensed?
  m_specifications.m_sensedGripper                 = false;

#ifdef BFR_FORCES_ENABLED
  // is you device actuated on the translation degrees of freedom?
  m_specifications.m_actuatedPosition              = true;
#else
  m_specifications.m_actuatedPosition              = false;
#endif

  // is your device actuated on the rotation degrees of freedom?
  m_specifications.m_actuatedRotation              = false;

  // is the gripper of your device actuated?
  m_specifications.m_actuatedGripper               = false;

  // can the device be used with the left hand?
  m_specifications.m_leftHand                      = true;

  // can the device be used with the right hand?
  m_specifications.m_rightHand                     = true;


  iters       = 0;    // Number of times through the control loop so far.
  enc0 = 0;
  enc1 = 0;
  enc2 = 0;
  m_jacobian.identity();


  /************************************************************************
        STEP 2:
        Here, you should implement code which tells the application if your
        device is actually connected to your computer and can be accessed.
        In practice this may be consist in checking if your I/O board
        is active or if your drivers are available.

        If your device can be accessed, set:
        m_systemAvailable = true;

        Otherwise set:
        m_systemAvailable = false;

        Your actual code may look like:

        bool result = checkIfMyDeviceIsAvailable()
        m_systemAvailable = result;
   *************************************************************************/

  m_deviceAvailable = sensorayio_.init();

  m_deviceReady = m_deviceAvailable;
}


//===========================================================================
/*!
    Destructor of cBFR1Device.
 */
//===========================================================================
cBFR1Device::~cBFR1Device()
{
  // close connection to device
  if (m_deviceReady)
  {
    close();
  }
}


//===========================================================================
/*!
    Open connection to your device.
 */
//===========================================================================
int cBFR1Device::open()
{
  // check if the system is available
  if (!m_deviceAvailable) return (-1);

  // if system is already opened then return
  if (m_deviceReady) return (0);

  /************************************************************************
        STEP 3:
        Here you need to implement code which open the connection to your
        device. This may include opening a connection to an interface board
        for instance or a USB port.

        If the connection succeeds, set the variable 'result' to true.
        otherwises, set the variable 'result' to false.

        Verify that your device is calibrated. If your device 
        needs calibration then call method calibrate() for wich you will 
        provide code in STEP 5 further bellow.
   *************************************************************************/

  //Not required. Already done in Step 2.
}


//===========================================================================
/*!
    Close connection to your device.
 */
//===========================================================================
int cBFR1Device::close()
{
  // check if the system has been opened previously
  if (!m_deviceReady) return (-1);

  /************************************************************************
        STEP 4:
        Here you need to implement code which closes the connection to your
        device.

        If the operation fails, simply set the variable 'result' to 0.
        If the connection succeeds, set the variable 'result' to any
        negative error value you may want to return. By default: -1.
   *************************************************************************/

  int result = 0;

  // *** INSERT YOUR CODE HERE ***
  sensorayio_.shutdown();                       // Shut down the driver
  result = 1;

  // update status
  m_deviceReady = false;

  return (result);
}


//===========================================================================
/*!
    Calibrate your device.
 */
//===========================================================================
int cBFR1Device::calibrate()
{
  /************************************************************************
        STEP 5:
        Here you shall implement code that handles a calibration procedure of the 
        device. In practice this may include initializing the registers of the
        encoder counters for instance. The method calibrate() is never explicitly 
        called by any higher level class from CHAI3D. A virtual tool will call 
        the method open() to create a connection between the computer and the
        haptic device, but will consider the device to be already calibrated.


        If the operation fails, you can return a negative error code such as
        -1 for instance.
        Otherwise return 0 if the operation succeeds.
   *************************************************************************/

  int error = 0;
  // *** INSERT YOUR CODE HERE ***
  // Not required.

  // error = calibrateMyDevice()

  return (error);
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \return  Returns the result
 */
//===========================================================================
unsigned int cBFR1Device::getNumDevices()
{
  /************************************************************************
        STEP 6:
        Here you may implement code which returns the number of available
        haptic devices of type "cBFR1Device" which are currently connected
        to your computer.

        In practice you will often have either 0 or 1 device. In which case
        the code here below is already implemented for you.

        If you have support more than 1 devices connected at the same time,
        then simply modify the code accordingly so that "numberOfDevices" takes
        the correct value.
   *************************************************************************/

  // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***

  int numberOfDevices;
  if (m_deviceAvailable)
  {
    // at least one device is available!
    numberOfDevices = 1;
  }
  else
  {
    // no devices are available
    numberOfDevices = 0;
  }

  // numberOfDevices = getNumberOfDevicesConnectedToTheComputer()

  return (numberOfDevices);
}


//===========================================================================
/*!
    Read the position of your device. Units are meters [m].
    \param  a_position  Return value.
    \return Return 0 if no error occurred.
 */
//===========================================================================
int cBFR1Device::getPosition(cVector3d& a_position)
{
  /************************************************************************
    STEP 7:
    Here you may implement code which reads the position (X,Y,Z) from
    your haptic device. Read the values from your device and modify
    the local variable (x,y,z) accordingly.
    If the operation fails return an error code such as -1 for instance.

    Note:
    For consistency, units must be in meters.
    If your device is located in front of you, the x-axis is pointing
    towards you (the operator). The y-axis points towards your right
    hand side and the z-axis points up towards the sky.
   *************************************************************************/

  bool flag;
  int error = 0;
  double x,y,z;

  // get encoder values
  if(sensorayio_.modeEncoderOnly())
  {
    flag = sensorayio_.readEncoders(enc0, enc1, enc2);
#ifdef DEBUG
    if(false == flag)
    { printf("\nError reading encoders"); }
    printf("Enc0: %ld, Enc1: %ld, Enc2: %ld \n", enc0, enc1, enc2);
#endif
  }

  // compute joint angles
  double ang0 = 2.0 * C_PI * (1.0 / HD_GEAR_RATIO_0) * (enc0 / HD_ENCODER_RESOLUTION_0);
  double ang1 = 2.0 * C_PI * (1.0 / HD_GEAR_RATIO_1) * (enc1 / HD_ENCODER_RESOLUTION_1);
  double ang2 =-2.0 * C_PI * (1.0 / HD_GEAR_RATIO_2) * (enc2 / HD_ENCODER_RESOLUTION_2);
#ifdef DEBUG
  printf("Ang0: %lf, Ang1: %lf, Ang2: %lf \n", cRadToDeg(ang0), cRadToDeg(ang1), cRadToDeg(ang2));
#endif

  // compute position in cartesian space
  double xg = -HD_LENGTH_0 * sin(ang0) + HD_LENGTH_2 * cos(ang1) + HD_LENGTH_1 * sin(ang2);
  double yg =  HD_LENGTH_2 * sin(ang1) + HD_LENGTH_1 * cos(ang2);
  double zg =  HD_LENGTH_0 * cos(ang0) + (HD_LENGTH_2 * cos(ang1) + HD_LENGTH_1 * sin(ang2)) * sin(ang0);
#ifdef DEBUG
  printf("Xraw: %lf, Xraw: %lf, Xraw: %lf \n", xg, yg, zg);
#endif

  // compute Jacobian
  m_jacobian(0,0) =-HD_LENGTH_0*cos(ang0);
  m_jacobian(0,1) =-HD_LENGTH_2*sin(ang1);
  m_jacobian(0,2) = HD_LENGTH_1*cos(ang2);

  m_jacobian(1,0) = 0.0;
  m_jacobian(1,1) = HD_LENGTH_2*cos(ang1);
  m_jacobian(1,2) =-HD_LENGTH_1*sin(ang2);

  m_jacobian(2,0) =-HD_LENGTH_0*sin(ang0) + (HD_LENGTH_2*cos(ang1) + HD_LENGTH_1*sin(ang2))*cos(ang0);
  m_jacobian(2,1) =-HD_LENGTH_2*sin(ang1)*sin(ang0);
  m_jacobian(2,2) = HD_LENGTH_1*cos(ang2)*sin(ang0);

  // Add offset if necessary to calibrate.
  // NOTE : The calibration might change with time, and so its magnitude is logged
  //        every time it is changed.
  x = xg - 1.275 - 0.175 + /** 2012-10-27 */ 0.205;
  y = yg - 0.485 - /** 2012-10-27 */ 0.01;
  z = zg - 0.127 - /** 2012-10-27 */ 0.025;
#ifdef DEBUG
  printf("Xcal: %lf, Xcal: %lf, Xcal: %lf \n", x, y, z);
#endif
  // store new position values
  a_position.set(x, y, z);

  // estimate linear velocity
  estimateLinearVelocity(a_position);

  // exit
  return (error);
}


//===========================================================================
/*!
    Read the orientation frame of your device end-effector

    \param  a_rotation  Return value.

    \return Return 0 if no error occurred.
 */
//===========================================================================
int cBFR1Device::getRotation(cMatrix3d& a_rotation)
{

  /************************************************************************
        STEP 7:
        Here you may implement code which reads the orientation frame from
        your haptic device. The orientation frame is expressed by a 3x3
        rotation matrix. The 1st column of this matrix corresponds to the
        x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
        The length of each column vector should be of length 1 and vectors need
        to be perpendicular to each other.
        If the operation fails return an error code such as -1 for instance.

        Note:
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        If your device has a stylus, make sure that you set the reference frame
        so that the x-axis corresponds to the axis of the stylus.

   *************************************************************************/

  int error = 0;
  double r00, r01, r02, r10, r11, r12, r20, r21, r22;
  cMatrix3d frame;
  frame.identity();


  // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***

  // if the device does not provide any rotation capabilities
  // we set the rotation matrix equal to the identiy matrix.
  r00 = 1.0;  r01 = 0.0;  r02 = 0.0;
  r10 = 0.0;  r11 = 1.0;  r12 = 0.0;
  r20 = 0.0;  r21 = 0.0;  r22 = 1.0;

  frame.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);

  // store new rotation matrix
  a_rotation = frame;

  // estimate angular velocity
  estimateAngularVelocity(a_rotation);

  // exit
  return (error);
}


//===========================================================================
/*!
    Read the gripper angle in radian.

    \param  a_angle  Return value.

    \return Return 0 if no error occurred.
 */
//===========================================================================
int cBFR1Device::getGripperAngleRAD(double& a_angle)
{

  /************************************************************************
        STEP 8:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as -1 for instance.

   *************************************************************************/
  int error = 0;

  // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***

  // return gripper angle
  a_angle = 0.0;  // a_angle = getGripperAngleFromMyDevice();

  // estimate gripper velocity
  estimateGripperVelocity(a_angle);

  // exit
  return (error);
}


//===========================================================================
/*!
    Send a force [N] to your haptic device

    \param  a_force  Force command to be applied to device.

    \return Return 0 if no error occurred.
 */
//===========================================================================
int cBFR1Device::setForce(const cVector3d& a_force)
{

#ifndef BFR_FORCES_ENABLED
  return 0;
#else
  /************************************************************************
        STEP 9:
        Here you may implement code which sends a force command (fx,fy,fz)
        to your haptic device.
        If the operation fails return an error code such as -1 for instance.

        Note:
        For consistency, units must be in Newtons.
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        For instance: if the force = (1,0,0), the device should move towards
        the operator, if the force = (0,0,1), the device should move upwards.

   *************************************************************************/

  int error = 0;

  if(sensorayio_.modeEncoderAndMotor())
  {
    // store new force value.
    m_prevForce = a_force;


    // *** INSERT YOUR CODE HERE ***

    // compute joint torques
    cVector3d torque = cMul(m_jacobian, a_force);

    // compute motor currents
    const double C_NM_PER_AMP = 0.1000; ///0.060;  // [Nm/A]
    const double C_CURENT_MAX_0 = 1.00;    //3.33;  // [A]
    const double C_CURENT_MAX_1 = 1.00;    //3.33;  // [A]
    const double C_CURENT_MAX_2 = 1.00;    //3.33;  // [A]

    double m_motorCurrent0 = torque(0) / (HD_GEAR_RATIO_0 * C_NM_PER_AMP);
    double m_motorCurrent1 = torque(1) / (HD_GEAR_RATIO_1 * C_NM_PER_AMP);
    double m_motorCurrent2 = torque(2) / (HD_GEAR_RATIO_2 * C_NM_PER_AMP);

    m_motorCurrent0 = cClamp(m_motorCurrent0, -C_CURENT_MAX_0, C_CURENT_MAX_0);
    m_motorCurrent1 = cClamp(m_motorCurrent1, -C_CURENT_MAX_1, C_CURENT_MAX_1);
    m_motorCurrent2 = cClamp(m_motorCurrent2, -C_CURENT_MAX_2, C_CURENT_MAX_2);

    // comput ouput voltage
    const double C_VOLT_PER_AMP = 1.0;
    double m_voltageLevel0 = C_VOLT_PER_AMP * m_motorCurrent0;
    double m_voltageLevel1 =-C_VOLT_PER_AMP * m_motorCurrent1;
    double m_voltageLevel2 =-C_VOLT_PER_AMP * m_motorCurrent2;

    //printf("Mot0: %lf, Mot1: %lf, Mot2: %lf \n", m_voltageLevel0, m_voltageLevel1, m_voltageLevel2);

    // send command to device (Analog out is device 1)
    flag = sensorayio_.readEncodersAndCommandMotors(enc0, enc1, enc2,
        m_voltageLevel0, m_voltageLevel1, m_voltageLevel2);
#ifdef DEBUG
    if(false == flag)
    { printf("\nError reading encoders and setting motors"); }
    printf("Enc0: %ld, Enc1: %ld, Enc2: %ld \n", enc0, enc1, enc2);
#endif
  }

  // exit
  return (error);
#endif
}


//===========================================================================
/*!
    Send a force [N] and torque [N*m] to the haptic device.

    \param  a_force Force command to be applied to device.
    \param  a_torque Torque command to be applied to device.

    \return  Return 0 if no error occurred.
 */
//===========================================================================
int cBFR1Device::setForceAndTorque(const cVector3d& a_force,
    const cVector3d& a_torque)
{
#ifndef BFR_FORCES_ENABLED
  return 0;
#else
    /************************************************************************
        STEP 10:
        Here you may implement code which sends a torque command (Tx,Ty,Tz)
        to your haptic device. This would be implemented if you have
        a haptic device with an active stylus for instance.
        If the operation fails return an error code such as -1 for instance.

        If your device does not support torque capabilities, you can simply 
        ignore this function. This method will simply ignore the torque component
        and simply send the force component to your device.

        Note that for optimization purposes, you may want to replace the
        following line:

        error = setForce(a_force);

        with code that directly applies force and torque components to your
        device.

        Note:
        For consistency, units must be in Newton meters for torques.
        A torque (1,0,0) would rotate counter clock-wise around the x-axis.
    *************************************************************************/

    int error = 0;

    // send force to the haptic device (see instructions above)
    error = setForce(a_force);

    // store new torque values
    m_prevTorque = a_torque;


    // *** INSERT YOUR CODE HERE ***

    // double tx = a_torque(0) ;
    // double ty = a_torque(1) ;
    // double tz = a_torque(2) ;
    // setTorqueToMyDevice(tx, ty, tz);


    // exit
    return (error);
#endif
}


//===========================================================================
/*!
    Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.

    \param  a_force  Force command.
    \param  a_torque  Torque command.
    \param  a_gripperForce  Gripper force command.

    \return Return 0 if no error occurred.
 */
//===========================================================================
int cBFR1Device::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
    const cVector3d& a_torque,
    const double a_gripperForce)
{
#ifndef BFR_FORCES_ENABLED
  return 0;
#else
    /************************************************************************
        STEP 11:
        Here you may implement code which sends a force command to the
        gripper of your haptic device.
        If the operation fails return an error code such as -1 for instance.

        If your device does not support gripper force capabilities, you can simply 
        ignore this function. This method will simply ignore the gripper command
        and simply send the force and torque components to your device.

        Note that for optimization purposes, you may want to replace the
        following line:

        error = setForceAndTorque(a_force, a_torque);

        with code that directly applies force, torque and gripper force 
        components to your device.
    *************************************************************************/

    int error = 0;

    // send force to the haptic device (see instructions above)
    error = setForceAndTorque(a_force, a_torque);

    // store new gripper torque value
    m_prevGripperForce = a_gripperForce;


    // *** INSERT YOUR CODE HERE ***

    // double torque = a_gripperTorque;

    // exit
    return (error);
#endif
}


//===========================================================================
/*!
    Read the status of the user switch [\b true = \e ON / \b false = \e OFF].

    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.

    \return Return 0 if no error occurred.
 */
//===========================================================================
int cBFR1Device::getUserSwitch(int a_switchIndex, bool& a_status)
{

  /************************************************************************
        STEP 12:
        Here you may implement code which reads the status of one or
        more user switches on your device. An application may request to read the status
        of a switch by passing its index number. The primary user switch mounted
        on the stylus of a haptic device will receive the index number 0. The
        second user switch is refered to as 1, and so on.

        The return value of a switch (a_status) shall be equal to \b true if the button
        is pressed or \b false otherwise.

        If the operation fails return an error code such as -1 for instance.

   *************************************************************************/

  int error = 0;

  // *** INSERT YOUR CODE HERE ***

  a_status = false;  // a_status = getUserSwitchOfMyDevice(a_switchIndex)

  return (error);
}

//---------------------------------------------------------------------------
#endif //C_ENABLE_BFR_DEVICE_SUPPORT
//---------------------------------------------------------------------------

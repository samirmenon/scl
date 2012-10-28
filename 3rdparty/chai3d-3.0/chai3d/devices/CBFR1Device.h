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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 793 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CBFR1DeviceH
#define CBFR1DeviceH

//---------------------------------------------------------------------------

#if defined(C_ENABLE_BFR_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

#include <sensoray/CSensoray3DofIODriver.hpp>

//===========================================================================
/*!
    \file       CBFR1Device.h

    \brief
    <b> Devices </b> \n 
    Custom Haptic Device (Template).
*/
//===========================================================================

//===========================================================================
/*!
    \class      cBFR1Device
    \ingroup    devices  

    \brief
    cBFR1Device provides a basic template which allows to very easily
    interface CHAI3D to your own custom haptic device. \n\n

    Simply follow the 12 commented step in file CBFR1Device.cpp
    and complete the code accordingly.
    Depending of the numbers of degrees of freedom of your device, not
    all methods may need to be implemented. For instance, if your device
    does not provide any rotation degrees of freedom, simply ignore
    the getRotation() method. Default values will be returned correctly
    if these are not implemented on your device. In the case of rotations
    for instance, the identity matrix is returned.\n\n

    You may also rename this class in which case you will also want to
    customize the haptics handler to automatically detect your device.
    Please consult method update() of the cHapticDeviceHandler class
    which is located in file CHapticDeviceHandler.cpp .
    Simply see how the haptic device handler already looks for
    device of type cBFR1Device.\n\n

    If you are encounting any problems with your implementation, check 
    for instance file cDeltaDevices.cpp which implement supports for the 
    delta(0)  and omega(0)  haptic devices. In order to verify the implementation
    use the 01-device example to get started. Example 11-effects is a great
    demo to verify how basic haptic effects may behave with you haptic
    devices. If you do encounter vibrations or instabilities, try reducing
    the maximum stiffness supported by your device 
    (see STEP1 in file CBFR1Device.cpp).\n
    
    Make  sure that your device is also communicating fast enough with 
    your computer. Ideally the communication period should take less 
    than 1ms in order to reach a desired update rate of at least 1000Hz.
    Problems can typicaly occur when using a slow serial port (RS232) for
    instance.\n
*/
//===========================================================================
class cBFR1Device : public cGenericHapticDevice
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cBFR1Device.
    cBFR1Device(unsigned int a_deviceNumber = 0);

    //! Destructor of cBFR1Device.
    virtual ~cBFR1Device();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Open connection to haptic device (0 indicates success).
    int open();

    //! Close connection to haptic device (0 indicates success).
    int close();

    //! Calibrate haptic device (0 indicates success).
    int calibrate();

    //! Returns the number of devices available for this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are meters [m].
    int getPosition(cVector3d& a_position);

    //! Read the orientation frame of the device end-effector.
    int getRotation(cMatrix3d& a_rotation);

    //! Read the gripper angle in radian.
    int getGripperAngleRAD(double& a_angle);

    //! Send a force [N] to the haptic device.
    int setForce(const cVector3d& a_force);

    //! Send a force [N] and a torque [N*m] to the haptic device.
    int setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque);

    //! Send a force [N] and a torque [N*m] and force [N] to the gripper.
    int setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);

    //! Read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    int getUserSwitch(int a_switchIndex, bool& a_status);


  protected:

    /********************************************************************
    If you need to declare any local variables or methods for your device,
    you may do it here. 
    *********************************************************************/
    //! Create a driver object for communicating with the sensoray board..
    sensoray::CSensoray3DofIODriver sensorayio_;

    /** Number of times through the control loop so far. */
    int   iters;
    /** Device Jacobian */
    cMatrix3d m_jacobian;
    /** To store the encoder positions */
    long enc0, enc1, enc2;
};

//---------------------------------------------------------------------------
#endif  // C_ENABLE_BFR_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

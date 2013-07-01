//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2013, CHAI3D.
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
    \author    Francois Conti
    \author    Force Dimension - www.forcedimension.com
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 931 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CSixenseDevicesH
#define CSixenseDevicesH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_SIXENSE_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

#define SIXENSE_BUTTON_BUMPER   (0x01<<7)
#define SIXENSE_BUTTON_JOYSTICK (0x01<<8)
#define SIXENSE_BUTTON_1        (0x01<<5)
#define SIXENSE_BUTTON_2        (0x01<<6)
#define SIXENSE_BUTTON_3        (0x01<<3)
#define SIXENSE_BUTTON_4        (0x01<<4)
#define SIXENSE_BUTTON_START    (0x01<<0)
#define SIXENSE_SUCCESS 0
#define SIXENSE_FAILURE -1
#define SIXENSE_MAX_CONTROLLERS 4


    typedef struct _sixenseControllerData 
    {
        float pos[3];
        float rot_mat[3][3];
        float joystick_x;
        float joystick_y;
        float trigger;
        unsigned int buttons;
        unsigned char sequence_number;
        float rot_quat[4];
        unsigned short firmware_revision;
        unsigned short hardware_revision;
        unsigned short packet_type;
        unsigned short magnetic_frequency;
        int enabled;
        int controller_index;
        unsigned char is_docked;
        unsigned char which_hand;
        unsigned char hemi_tracking_enabled;
    } sixenseControllerData;

    typedef struct _sixenseAllControllerData
    {
        sixenseControllerData controllers[4];
    } sixenseAllControllerData;

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    \file       CSixenseDevices.h

    \brief
    <b> Devices </b> \n 
    Sixense Devices.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cSixenseDevice
    \ingroup    devices  

    \brief
    Interface to Sixsense tracking devices.

    \brief  
    cSixenseDevice implements an interface to Sixense interface, in particular 
    the Razor Hydra magnetic tracker.
*/
//==============================================================================
class cSixenseDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

 public:

    //! Constructor of cSixenseDevice.
    cSixenseDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cSixenseDevice.
    virtual ~cSixenseDevice();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

 public:

    //! Open connection to device.
    virtual bool open();

    //! Close connection to device.
    virtual bool close();

    //! Calibrate device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! Read position of device. Units are meters [m].
    virtual bool getPosition(cVector3d& a_position);

    //! Read orientation frame (3x3 matrix) of the device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! Read gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! Read status of user switch [__true__ = __ON__ / __false__ = __OFF__].
    bool getUserSwitch(int a_switchIndex, bool& a_status);


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! Get number of haptic devices available for this class of devices.
    static unsigned int getNumDevices();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - DEVICE LIBRARY INITIALIZATION:
    //--------------------------------------------------------------------------

protected:

    //! Open libraries for this class of devices.
    static bool openLibraries();

    //! Close libraries for this class of devices.
    static bool closeLibraries();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - DEVICE LIBRARIES:
    //--------------------------------------------------------------------------

protected:

    //! Allocation table for devices of this class. __true__ means that the device has been allocated, __false__ means free.
    static bool s_allocationTable[C_MAX_DEVICES];

    //! Number of instances for this class of devices currently using the libraries.
    static unsigned int s_libraryCounter;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - INTERNAL:
    //--------------------------------------------------------------------------
    
protected:

    //! Time guard for data acquisition.
    static cPrecisionClock m_timeguard;

    //! Data acquired from the controller.
    static sixenseAllControllerData m_data;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - INTERNAL:
    //--------------------------------------------------------------------------

protected:

    //! Update data from controllers.
    bool updateData();
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //_DISABLE_SIXENSE_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

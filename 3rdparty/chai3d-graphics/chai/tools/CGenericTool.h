//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \author    Federico Barbagli
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericToolH
#define CGenericToolH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGenericTool.h

    \brief  
    <b> Haptic Tools </b> \n 
    Base Class.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGenericTool
    \ingroup    tools  

    \brief      
    cGenericTool describes a generic class to create virtual tools inside a 
    virtual environment (cWorld) and connecting them to haptic devices.
*/
//===========================================================================
class cGenericTool : public cGenericObject
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGenericTool.
    cGenericTool();

    //! Destructor of cGenericTool.
    virtual ~cGenericTool() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Connect this tool to a haptic device.
    void setHapticDevice(cGenericHapticDevice* a_device) { if (a_device != NULL) { m_device = a_device; } }

    //! Get the handle of haptic device to wich this tool is connected to.
    cGenericHapticDevice* getHapticDevice() { return (m_device); }

    //! Render the object in OpenGL.
    virtual void render(const int a_renderMode=0) {};

    //! Update Position, orientation, velocity and other degree of freedoms of tool.
    virtual void updatePose() {};

    //! Compute interaction forces with environment.
    virtual void computeInteractionForces() {};

    //! Apply latest forces to device.
    virtual void applyForces() {};

    //! Start communication with the device connected to the tool (0 indicates success).
    virtual int start() { return (-1); }

    //! Stop communication with the device connected to the tool (0 indicates success).
    virtual int stop() { return (-1); }

    //! Initialize encoders on device connected to the tool (0 indicates success).
    virtual int initialize(const bool a_resetEncoders=false) { return (-1); }

    //! Toggle forces \b ON.
    virtual int setForcesON()  { return (-1); }

    //! Toggle forces \b OFF.
    virtual int setForcesOFF() { return (-1); }

    //! Read the status of one of the switches on this device.
    virtual bool getUserSwitch(int a_switchIndex);

    //! Check if the tool is touching a particular object.
    virtual bool isInContact(cGenericObject* a_object) { return (false); }


  protected:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Handle to the haptic device driver.
    cGenericHapticDevice *m_device;

    //! Status of the user switches of the device attached to this tool.
    int m_userSwitches;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


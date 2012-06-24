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
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 846 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericToolH
#define CGenericToolH
//---------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "devices/CGenericHapticDevice.h"
#include "forces/CAlgorithmFingerProxy.h"
#include "forces/CAlgorithmPotentialField.h"
#include "world/CGenericObject.h"
#include "world/CWorld.h"
#include "tools/CHapticPoint.h"
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
    cGenericTool describes a base class to create virtual tools inside a 
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
    cGenericTool(cWorld* a_parentWorld);

    //! Destructor of cGenericTool.
    virtual ~cGenericTool() {};


    //-----------------------------------------------------------------------
    // METHODS - SETUP
    //-----------------------------------------------------------------------

    //! Return the parent world to which the tool belongs too.
    cWorld* getParentWorld() { return (m_parentWorld); }

    //! Define a haptic device that will take control of the current tool.
    void setHapticDevice(cGenericHapticDevice* a_hapticDevice) { if (a_hapticDevice != NULL) { m_hapticDevice = a_hapticDevice; } }

    //! Retrieve the haptic device that is controlling the tool.
    cGenericHapticDevice* getHapticDevice() { return (m_hapticDevice); }

    //! Start communication with the haptic device. Initialize the tool (0 indicates success).
    virtual int start();

    //! Stop communication with the haptic device. (0 indicates success).
    virtual int stop();


    //-----------------------------------------------------------------------
    // METHODS - HAPTIC DEVICE
    //-----------------------------------------------------------------------

    //! Read the status of one of the user switches of the haptic device.
    virtual bool getUserSwitch(int a_switchIndex);

    //! Read the position of haptic device expressed in device local coordinates.
    virtual cVector3d getDeviceLocalPos() { return (m_deviceLocalPos); }

    //! Read the position of haptic device expressed in world global coordinates.
    virtual cVector3d getDeviceGlobalPos() { return (m_deviceGlobalPos); }

    //! Read the orientation of haptic device expressed in device local coordinates.
    virtual cMatrix3d getDeviceLocalRot() { return (m_deviceLocalRot); }

    //! Read the orientation of haptic device expressed in world global coordinates.
    virtual cMatrix3d getDeviceGlobalRot() { return (m_deviceGlobalRot); }

    //! Read the linear velocity of the device expressed in device local coordinates.
    virtual cVector3d getDeviceLocalLinVel() { return (m_deviceLocalLinVel); }

    //! Read the linear velocity of the device expressed in world global coordinates.
    virtual cVector3d getDeviceGlobalLinVel() { return (m_deviceGlobalLinVel); }

    //! Read the angular velocity of the device expressed in device local coordinates.
    virtual cVector3d getDeviceLocalAngVel() { return (m_deviceLocalAngVel); }

    //! Read the angular velocity of the device expressed in world global coordinates.
    virtual cVector3d getDeviceGlobalAngVel() { return (m_deviceGlobalAngVel); }

    //! Read a transformation matrix which expressed the position and orientation of the haptic device in the local tool frame.
    virtual cTransform getDeviceLocalTransform() { return (cTransform(m_deviceLocalPos, m_deviceLocalRot)); }

    //! Read a transformation matrix which expressed the position and orientation of the haptic device in world coordinates
    virtual cTransform getDeviceGlobalTransform() { return (cTransform(m_deviceGlobalPos, m_deviceGlobalRot)); }


    //-----------------------------------------------------------------------
    // METHODS - FORCE RENDERING
    //-----------------------------------------------------------------------

    //! Initialize the contact force models of the tool to its current position.
    virtual void initialize();

    //! Update Position, orientation, velocity and other degree of freedoms of tool by reading sensor data from the haptic device.
    virtual void updatePose();

    //! Update the position and orientation of the tool image.
    virtual void updateToolImagePosition();

    //! Compute all interaction forces between the tool interaction points and the virtual environment.
    virtual void computeInteractionForces();

    //! Send the latest computed interaction forces and torques to the haptic device.
    virtual void applyForces();

    //! Toggle forces to the haptic device \b ON.
    virtual int setForcesON();

    //! Toggle forces to the haptic device \b OFF.
    virtual int setForcesOFF();

    //! Enable or disable the dynamic proxy algorithm to support dynamic environements where objects move.
    virtual void enableDynamicObjects(bool a_enabled);


    //-----------------------------------------------------------------------
    // METHODS - INTERACTION POINTS
    //-----------------------------------------------------------------------

    //! Check if the tool is touching a particular object.
    virtual bool isInContact(cGenericObject* a_object);

    //! Get number of interaction points composing this tool.
    int getNumInteractionPoints() { return (int)(m_hapticPoints.size()); }

    //! Get pointer to interaction points by passing the index number
    cHapticPoint* getInteractionPoint(int a_index) { return (m_hapticPoints[a_index]); }


    //-----------------------------------------------------------------------
    // METHODS - TOOL SETTINGS
    //-----------------------------------------------------------------------

    //! Set the translational virtual workspace dimensions in which tool will be working.
    void setWorkspaceRadius(const double& a_workspaceRadius);

    //! Read the radius of the workspace of the tool.
    double getWorkspaceRadius() { return(m_workspaceRadius); }

    //! Set the scale factor applied on the haptic device to compute the position of the tool.
    void setWorkspaceScaleFactor(const double& a_workspaceScaleFactor);

    //! Read the current scale factor between the haptic device and the tool.
    double getWorkspaceScaleFactor() { return (m_workspaceScaleFactor); }

    //! Set radius size of contact points.
    virtual void setRadius(double a_radius);

    //! Set radius size of contact points.
    virtual void setRadius(double a_radiusDisplay, double a_radiusContact);

    //! Set radius size of the of the physical proxy used in all contact points.
    virtual void setRadiusContact(double a_radiusContact);

    //! Set radius size of the spheres used to display the proxies and goals positions
    virtual void setRadiusDisplay(double a_radiusDisplay); 

    //! If set to \b true, then the tool waist for the simulation to compute a small force before sending it to the haptic device. This feature typically avoids having the device "jump" at the initialoization of an application.
    inline void setWaitForSmallForce(bool a_value) { m_waitForSmallForce = a_value; }

    //! Get tool state about force handling.
    inline bool getWaitForSmallForce() { return (m_waitForSmallForce); }


    //-----------------------------------------------------------------------
    // METHODS - GRAPHICS
    //-----------------------------------------------------------------------

    //! Set display properties of contact points.
    virtual void setShowContactPoints(bool a_showProxy = true, 
                                      bool a_showGoal = false, 
                                      cColorf a_colorLine = cColorf(0.5, 0.5, 0.5));

    //! Render the tool graphicaly in OpenGL.
    virtual void render(cRenderOptions& a_options) {};


    //-----------------------------------------------------------------------
    // MEMBERS 
    //-----------------------------------------------------------------------

    //! Mesh model that can be used to image any 3D model of a tool. 
    cMesh* m_image;

    //! Contact points composing the tool.
    vector <cHapticPoint*> m_hapticPoints;


    //-----------------------------------------------------------------------
    // MEMBERS - HAPTIC DEVICE:
    //-----------------------------------------------------------------------

  protected:

    //! Handle to the haptic device.
    cGenericHapticDevice *m_hapticDevice;

    //! Status of the user switches of the device attached to the tool.
    int m_userSwitches;

    //! If \b true then the tool has been started and is enabled. \b false otherwise.
    bool m_enabled;


    //-----------------------------------------------------------------------
    // MEMBERS - COMPUTED INTERACTION FORCES:
    //-----------------------------------------------------------------------

  public: 

    //! Last computed interaction force [N] in world global coordinates. 
    cVector3d m_lastComputedGlobalForce;

    //! Last computed interaction torque [N*m] in world global coordinates. 
    cVector3d m_lastComputedGlobalTorque;

    //! Last computed interaction gripper force [N] in device local coordinates. 
    double m_lastComputedGripperForce;


  protected:

    //! Last computed interaction force [N] in device local coordinates. 
    cVector3d m_lastComputedLocalForce;

    //! Last computed interaction torque [N*m] in device local coordinates. 
    cVector3d m_lastComputedLocalTorque;


    //-----------------------------------------------------------------------
    // MEMBERS - TOOL POSE AND WORKSPACE SETTINGS
    //-----------------------------------------------------------------------

  protected:

    //! Radius of the workspace which can be accessed by the tool.
    double m_workspaceRadius;

    //! Scale factor applied on the haptic device to compute the position of the tool.
    double m_workspaceScaleFactor;


  public:

    //! Position of the haptic device in device local coordinates. This value is read only!
    cVector3d m_deviceLocalPos;

    //! Position of the haptic device in world global coordinates. This value is read only!
    cVector3d m_deviceGlobalPos;

    //! Orientation of the haptic device in device local coordinates. This value is read only!
    cMatrix3d m_deviceLocalRot;

    //! Orientation of the haptic device in world global coordinates. This value is read only!
    cMatrix3d m_deviceGlobalRot;

    //! Linear velocity of the haptic device in device local coordinates. This value is read only!
    cVector3d m_deviceLocalLinVel;

    //! Linear velocity of the haptic device in world global coordinates. This value is read only!
    cVector3d m_deviceGlobalLinVel;

    //! Angular velocity of the haptic device in device local coordinates. This value is read only!
    cVector3d m_deviceLocalAngVel;

    //! Angular velocity of the haptic device in world global coordinates. This value is read only!
    cVector3d m_deviceGlobalAngVel;

    //! Gripper angle in radians. This value is read only!
    double m_deviceGripperAngle;


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

  protected:

    //! World in which tool is interacting in.
    cWorld* m_parentWorld;

    //! Last status of user switch 0. This value is used by the graphical rendering function.
    bool m_userSwitch0;

    //! This flag records whether the user has enabled forces.
    bool m_forceON;

    //! Flag to avoid initial bumps in force (has the user sent a _small_ force yet?).
    bool m_forceStarted;

    /*!
        Normally this class waits for a very small force before initializing forces
        to avoid initial "jerks" (a safety feature); you can bypass that requirement
        with this variable.
    */
    bool m_waitForSmallForce;

    //! Counter which waits for up to three cycles of small forces before enabling force.
    int m_waitForSmallForceCounter;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


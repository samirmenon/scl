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
    \version   2.0.0 $Rev: 270 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "tools/CGeneric3dofPointer.h"
#include "graphics/CTriangle.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
    Constructor of cGeneric3dofPointer.

    \fn       cGeneric3dofPointer::cGeneric3dofPointer(cWorld* a_world)
    \param    a_world  World in which the tool will operate.
*/
//===========================================================================
cGeneric3dofPointer::cGeneric3dofPointer(cWorld* a_world)
{
    // this makes sure that we do not send a huge force to the device
    // as soon as the demo starts.
    m_waitForSmallForce = true;

    // default radius of tip of tool
    m_displayRadius = 0.001;

    // set world
    m_world = a_world;

    // set a default device for the moment
    m_device = new cGenericHapticDevice();


    //-------------------------------------------------------------------
    // FORCE MODELS SUPPORTED BY THE TOOL
    //-------------------------------------------------------------------

    // create a default proxy algorithm force renderer
    m_proxyPointForceModel = new cProxyPointForceAlgo();

    // initialize proxy model
    m_proxyPointForceModel->initialize(m_world, cVector3d(0,0,0));
    m_proxyPointForceModel->setProxyRadius(m_displayRadius);

    // create a default potential field force renderer
    m_potentialFieldsForceModel = new cPotentialFieldForceAlgo();

    // initialize potential field model
    m_potentialFieldsForceModel->initialize(m_world, cVector3d(0,0,0));

    // force settings
    m_forceON = true;
    m_forceStarted = false;


    //-------------------------------------------------------------------
    // GRAPHICAL MODEL OF THE TOOL
    //-------------------------------------------------------------------

    // set the default material properties for the sphere representing  the proxy
    m_materialProxy.m_ambient.set(0.7, 0.7, 0.7);
    m_materialProxy.m_diffuse.set(0.9, 0.9, 0.9);
    m_materialProxy.m_specular.set(1.0, 1.0, 1.0);

    // set the default material properties for proxy when the user switch is engaged
    m_materialProxyButtonPressed.m_ambient.set(0.3, 0.3, 0.3);
    m_materialProxyButtonPressed.m_diffuse.set(0.5, 0.5, 0.5);
    m_materialProxyButtonPressed.m_specular.set(1.0, 1.0, 1.0);

    // set the default material properties for the sphere representing the device
    cMaterial materialDevice;
    materialDevice.m_ambient.set(0.6, 0.6, 0.6);
    materialDevice.m_diffuse.set(0.8, 0.8, 0.8);
    materialDevice.m_specular.set(1.0, 1.0, 1.0);

    // Sphere representing the device
    m_deviceSphere = new cShapeSphere(0.019);
    m_deviceSphere->m_material = materialDevice;
    m_deviceSphere->setHapticEnabled(false);
    addChild(m_deviceSphere);

    // Sphere representing the proxy
    m_proxySphere = new cShapeSphere(0.020);
    m_proxySphere->setHapticEnabled(false);
    m_proxySphere->m_material = m_materialProxy;
    addChild(m_proxySphere);

    // Mesh representing the device
    m_deviceMesh = new cMesh(m_world);
    m_deviceMesh->setHapticEnabled(false);
    m_deviceSphere->addChild(m_deviceMesh);

    // Mesh representing the proxy
    m_proxyMesh = new cMesh(m_world);
    m_proxyMesh->setHapticEnabled(false);
    m_proxySphere->addChild(m_proxyMesh);

    // Default color of the line which connects the device and the proxy spheres
    m_colorLine.set(0.7f, 0.7f, 0.7f);

    // This sets both the rendering radius and the actual proxy radius
    setRadius(0.05);


    //-------------------------------------------------------------------
    // WORKSPACE AND POSITION
    //-------------------------------------------------------------------

    // set workspace size
    m_workspaceRadius = 1.0;
    m_workspaceScaleFactor = 1.0;

    // init device related variables
    m_deviceGlobalPos.zero();
    m_deviceLocalPos.zero();
    m_lastComputedLocalForce.zero();
    m_lastComputedGlobalForce.zero();
    m_deviceLocalVel.zero();
    m_deviceGlobalVel.zero();
}


//==========================================================================
/*!
    Destructor of cGeneric3dofPointer.

    \fn       cGeneric3dofPointer::~cGeneric3dofPointer()
*/
//===========================================================================
cGeneric3dofPointer::~cGeneric3dofPointer()
{
    // check if device is available
    if (m_device == NULL) { return; }

    // close device driver
    m_device->close();

    delete m_device;
}


//==========================================================================
/*!
    Initialize device

    \fn       void cGeneric3dofPointer::initialize(const bool a_resetEncoders=false)
    \param    a_resetEncoders  If true, this resets the device's 0 position
                to the current position (if this device supports re-zero'ing).
                That means that if your device supports re-zero'ing (e.g. Phantom
                premiums), you should be careful about calling this function
                in the middle of your program with a_resetEncoders set to 'true'.
                Or if you do call it with a_resetEncoders set to true, you should
                make sure your user knows to hold the Phantom in place.
    \return   0 indicates success, non-zero indicates an error
*/
//===========================================================================
int cGeneric3dofPointer::initialize(const bool a_resetEncoders)
{
    // check if device is available
    if (m_device == NULL)
    {
        return (-1);
    }

    // initialize all force models
    m_proxyPointForceModel->initialize(m_world, m_deviceGlobalPos);
    m_potentialFieldsForceModel->initialize(m_world, m_deviceGlobalPos);

	// if device encoders need to be initialize, call initialize function here.
	// otherwise normal initialization is perform in the start() function
	if (a_resetEncoders)
	{
		m_device->initialize(true);
	}

    // exit
    return 0;
}


//==========================================================================
/*!
    Starts communication with the haptic device.

    \fn       void cGeneric3dofPointer::start()
    \return   0 indicates success, non-zero indicates an error
*/
//===========================================================================
int cGeneric3dofPointer::start()
{
    // check if device is available
    if (m_device == NULL)
    {
        return -1;
    }

	int result = 0;

    // open connection to device
    result = m_device->open();

	// initialize device
	if (result == 0)
	{
		result = m_device->initialize();
	}

	// resturn result
	return (result);
}


//==========================================================================
/*!
    Stop system. Apply zero force to device

    \fn       void cGeneric3dofPointer::stop()
    \return   0 indicates success, non-zero indicates an error
*/
//===========================================================================
int cGeneric3dofPointer::stop()
{
    // check if device is available
    if (m_device == NULL) { return -1; }

    // stop the device
    return m_device->close();
}


//==========================================================================
/*!
    Sets the virtual volume in which the virtual tool will be moving.

    \fn       void cGeneric3dofPointer::setWorkspaceRadius(const double& a_workspaceRadius)
    \param    a_workspaceRadius   Radius of the workspace.
*/
//===========================================================================
void cGeneric3dofPointer::setWorkspaceRadius(const double& a_workspaceRadius)
{
    // update new workspace size
    m_workspaceRadius = a_workspaceRadius;

    // compute the new scale factor between the workspace of the tool
    // and one of the haptic device
    if (m_device != NULL)
    {
        m_workspaceScaleFactor = m_workspaceRadius / m_device->getSpecifications().m_workspaceRadius;
    }
    else
    {
        m_workspaceScaleFactor = 1.0;
    }
}


//==========================================================================
/*!
    Define a scale factor between the physical workspace of the haptic device
    and the workspace span by the virtual tool.

    \fn       void cGeneric3dofPointer::setWorkspaceScaleFactor(const double& a_workspaceScaleFactor)
    \param    a_workspaceScaleFactor   Workspace scale factor.
*/
//===========================================================================
void cGeneric3dofPointer::setWorkspaceScaleFactor(const double& a_workspaceScaleFactor)
{
    // make sure that input value is bigger than zero
    double value = cAbs(a_workspaceScaleFactor);
    if (value == 0) { return; }

    // update scale factor
    m_workspaceScaleFactor = value;

    // compute the new scale factor between the workspace of the tool
    // and one of the haptic device
    m_workspaceRadius =  m_workspaceScaleFactor * m_device->getSpecifications().m_workspaceRadius;
}


//==========================================================================
/*!
    Update position of pointer and orientation of wrist.

    \fn       void cGeneric3dofPointer::updatePose()
*/
//===========================================================================
void cGeneric3dofPointer::updatePose()
{
    cVector3d pos, vel;

    // check if device is available
    if (m_device == NULL) { return; }

    // read device position
    m_device->getPosition(pos);

    // adjust for tool workspace scale factor
    m_deviceLocalPos = m_workspaceScaleFactor * pos;

    // update global position of tool
    cVector3d tPos;
    m_globalRot.mulr(m_deviceLocalPos, tPos);
    tPos.addr(m_globalPos, m_deviceGlobalPos);

    // read device orientation
    m_device->getRotation(m_deviceLocalRot);

    // update global orientation of tool
    m_deviceLocalRot.mulr(m_globalRot, m_deviceGlobalRot);

    // read switches
    m_userSwitch0 = getUserSwitch(0);

    // read velocity of the device in local coordinates
    m_device->getLinearVelocity(vel);

    // adjust for tool workspace scale factor
    m_deviceLocalVel = m_workspaceScaleFactor * vel;

    // update global velocity of tool
    m_globalRot.mulr(m_deviceLocalVel, m_deviceGlobalVel);
}


//===========================================================================
/*!
    Compute the interaction forces between the tool and the virtual
    object inside the virtual world.

    \fn       void cGeneric3dofPointer::computeInteractionForces()
*/
//===========================================================================
void cGeneric3dofPointer::computeInteractionForces()
{
    // temporary variable to store forces
    cVector3d force;
    force.zero();

    // compute forces in world coordinates for each point force algorithm
    force.add(m_potentialFieldsForceModel->computeForces(m_deviceGlobalPos, m_deviceGlobalVel));
    force.add(m_proxyPointForceModel->computeForces(m_deviceGlobalPos, m_deviceGlobalVel));

    // copy result
    m_lastComputedGlobalForce.copyfrom(force);
}


//==========================================================================
/*!
    Apply the latest computed force to the device.

    \fn       void cGeneric3dofPointer::applyForces()
*/
//===========================================================================
void cGeneric3dofPointer::applyForces()
{
    // check if device is available
    if (m_device == NULL)
    {
        return;
    }

    // convert force into device local coordinates
    cMatrix3d tRot;
    m_globalRot.transr(tRot);
    tRot.mulr(m_lastComputedGlobalForce, m_lastComputedLocalForce);

    if (
        (m_waitForSmallForce == false)  ||
        ((!m_forceStarted) && (m_lastComputedLocalForce.lengthsq() <0.000001))
      )
    {
        m_forceStarted = true;
    }

    // send force to device
    if ((m_forceON) && (m_forceStarted))
    {
        m_device->setForce(m_lastComputedLocalForce);
    }
    else
    {
        cVector3d zeroForce(0.0, 0.0, 0.0);
        m_device->setForce(zeroForce);
    }
}


//==========================================================================
/*!
    Render the current tool in OpenGL.

    \fn       void cGeneric3dofPointer::render(const int a_renderMode=0)
    \param    a_renderMode  rendering mode; see cGenericObject.cpp.
*/
//===========================================================================
void cGeneric3dofPointer::render(const int a_renderMode)
{
    // If multipass transparency is enabled, only render on a single
    // pass...
    if (a_renderMode != CHAI_RENDER_MODE_NON_TRANSPARENT_ONLY && a_renderMode != CHAI_RENDER_MODE_RENDER_ALL)
    return;

    // compute local position of proxy
    cVector3d proxyLocalPos;
    cMatrix3d tRot;
    proxyLocalPos = m_proxyPointForceModel->getProxyGlobalPosition();
    proxyLocalPos.sub(m_globalPos);
    m_globalRot.transr(tRot);
    tRot.mul(proxyLocalPos);

    // update position information of graphic entity for the device
    m_deviceSphere->setPos(m_deviceLocalPos);
    m_deviceSphere->setRot(m_deviceLocalRot);

    // update position information of graphic entity for the proxy
    m_proxySphere->setPos(proxyLocalPos);
    m_proxySphere->setRot(m_deviceLocalRot);

    // Button 0 determines the color of the proxy
    if (m_userSwitch0)
    {
        m_proxySphere->m_material = m_materialProxyButtonPressed;
    }
    else
    {
        m_proxySphere->m_material = m_materialProxy;
    }

    // if proxy and device sphere are enabled, draw
    if ((m_proxySphere->getShowEnabled()) && (m_deviceSphere->getShowEnabled()))
    {
        glDisable(GL_LIGHTING);
        glLineWidth(1.0);
        glColor4fv(m_colorLine.pColor());
        glBegin(GL_LINES);
            glVertex3d(m_deviceLocalPos.x, m_deviceLocalPos.y, m_deviceLocalPos.z);
            glVertex3d(proxyLocalPos.x, proxyLocalPos.y, proxyLocalPos.z);
        glEnd();
        glEnable(GL_LIGHTING);
    }
}


//==========================================================================
/*!
    Set the radius of the proxy. The value passed as parameter corresponds
    to the size of the sphere which is rendered graphically. The physical
    size of the proxy, one which collides with the triangles is set to
    CHAI_SCALE_PROXY_RADIUS * a_radius.

    \fn       void cGeneric3dofPointer::setRadius(const double& a_radius)
    \param    a_radius  radius of pointer.
*/
//===========================================================================
void cGeneric3dofPointer::setRadius(const double& a_radius)
{
    // update the radius that's rendered
    m_displayRadius = a_radius;

    // update the radius used for collision detection
    m_proxyPointForceModel->setProxyRadius(a_radius);

    // update radius of display sphere.
    // make the device sphere slightly smaller to avoid
    // graphical artifacts.
    m_proxySphere->setRadius(1.00 * a_radius);
    m_deviceSphere->setRadius(0.99 * a_radius);
}


//==========================================================================
/*!
    Turns forces \b ON.

    \fn     void cGeneric3dofPointer::setForcesON()
    \return   0 indicates success, non-zero indicates an error

*/
//===========================================================================
int cGeneric3dofPointer::setForcesON()
{
    if (!m_forceON)
    {
          m_forceStarted = false;
          m_forceON = true;
    }
    return 0;
}


//==========================================================================
/*!
    Turns forces \b OFF.

    \fn       void cGeneric3dofPointer::setForcesOFF()
    \return   0 indicates success, non-zero indicates an error
*/
//===========================================================================
int cGeneric3dofPointer::setForcesOFF()
{
    m_forceON = false;
    m_lastComputedLocalForce.zero();
    m_lastComputedGlobalForce.zero();
    applyForces();
    return 0;
}


//==========================================================================
/*!
    Check if the tool is currently interacting with the given object.

    \fn       bool cGeneric3dofPointer::isInContact(cGenericObject* a_object)
    \return   Return \e true if the tool is interacting with the object
*/
//===========================================================================
bool cGeneric3dofPointer::isInContact(cGenericObject* a_object)
{
    //-------------------------------------------------------------------
    // CHECK PROXY ALGORITHM
    //-------------------------------------------------------------------

    // contact 0
    if (m_proxyPointForceModel->m_contactPoint0->m_object == a_object)
    {
        return (true);
    }

    // contact 1
    if ((m_proxyPointForceModel->m_contactPoint0->m_object != NULL) &&
        (m_proxyPointForceModel->m_contactPoint1->m_object == a_object))
    {
        return (true);
    }

    // contact 2
    if ((m_proxyPointForceModel->m_contactPoint0->m_object != NULL) &&
        (m_proxyPointForceModel->m_contactPoint1->m_object != NULL) &&
        (m_proxyPointForceModel->m_contactPoint2->m_object == a_object))
    {
        return (true);
    }

    //-------------------------------------------------------------------
    // CHECK POTENTIAL FIELD ALGORITHM
    //-------------------------------------------------------------------
    unsigned int num = m_potentialFieldsForceModel->m_interactionRecorder.m_interactions.size();
    unsigned int i = 0;
    while (i < num)
    {
        // check next interaction
        if (m_potentialFieldsForceModel->m_interactionRecorder.m_interactions[i].m_object == a_object)
        {
            return (true);
        }

        // increment counter
        i++;
    }

    // no object in contact
    return (false);
}


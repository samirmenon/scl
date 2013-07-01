//===========================================================================
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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1067 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCHAI3DH
#define CCHAI3DH
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       chai3d.h
    
    \brief    
    <b> CHAI3D </b> \n
    Main Header File.
*/
//===========================================================================

//---------------------------------------------------------------------------
//!     \defgroup   devices  Devices
//---------------------------------------------------------------------------
#include "devices/CGenericDevice.h"
#include "devices/CGenericHapticDevice.h"
#include "devices/CHapticDeviceHandler.h"
#include "devices/CMyCustomDevice.h"
#include "devices/CDeltaDevices.h"     
#include "devices/CPhantomDevices.h"
#include "devices/CSixenseDevices.h"


//---------------------------------------------------------------------------
//!     \defgroup   graphics  Graphics 
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CDisplayList.h"
#include "graphics/CDraw3D.h"
#include "graphics/CEdge.h"
#include "graphics/CFog.h"
#include "graphics/CFont.h"
#include "graphics/CImage.h"
#include "graphics/CPrimitives.h"
#include "graphics/CRenderOptions.h"
#include "graphics/CTriangle.h"
#include "graphics/CVertex.h"


//---------------------------------------------------------------------------
//!     \defgroup   materials  Material Properties 
//---------------------------------------------------------------------------
#include "materials/CGenericTexture.h"
#include "materials/CMaterial.h"
#include "materials/CTexture1d.h"
#include "materials/CTexture2d.h"


//---------------------------------------------------------------------------
//!     \defgroup   math  Math 
//---------------------------------------------------------------------------
#include "math/CConstants.h"
#include "math/CMaths.h"
#include "math/CMatrix3d.h"
#include "math/CQuaternion.h"
#include "math/CTransform.h"
#include "math/CVector3d.h"


//---------------------------------------------------------------------------
//!     \defgroup   widgets  Widgets
//---------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
#include "widgets/CBackground.h"
#include "widgets/CBitmap.h"
#include "widgets/CDial.h"
#include "widgets/CLabel.h"
#include "widgets/CLevel.h"
#include "widgets/CPanel.h"
#include "widgets/CScope.h"


//---------------------------------------------------------------------------
//!     \defgroup   world  World
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "world/CMesh.h"
#include "world/CMultiMesh.h"
#include "world/CShapeBox.h"
#include "world/CShapeCylinder.h"
#include "world/CShapeLine.h"
#include "world/CShapeSphere.h"
#include "world/CShapeTorus.h"
#include "world/CWorld.h"


//---------------------------------------------------------------------------
//!     \defgroup   display  Camera
//---------------------------------------------------------------------------  
#include "display/CCamera.h"


//---------------------------------------------------------------------------
//!     \defgroup   lighting  Lighting Properties 
//---------------------------------------------------------------------------
#include "lighting/CGenericLight.h"
#include "lighting/CDirectionalLight.h"
#include "lighting/CPositionalLight.h"
#include "lighting/CSpotLight.h"
#include "lighting/CShadowMap.h"


//---------------------------------------------------------------------------
//!     \defgroup   tools  Haptic Tools
//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
#include "tools/CHapticPoint.h"
#include "tools/CToolCursor.h"
#include "tools/CToolGripper.h"


//---------------------------------------------------------------------------
//!     \defgroup   effects  Haptic Effects
//---------------------------------------------------------------------------
#include "effects/CGenericEffect.h"
#include "effects/CEffectMagnet.h"
#include "effects/CEffectSurface.h"
#include "effects/CEffectStickSlip.h"
#include "effects/CEffectViscosity.h"
#include "effects/CEffectVibration.h"


//---------------------------------------------------------------------------
//!     \defgroup   forces  Force Rendering Algorithms
//---------------------------------------------------------------------------
#include "forces/CGenericForceAlgorithm.h"
#include "forces/CAlgorithmFingerProxy.h"
#include "forces/CAlgorithmPotentialField.h"
#include "forces/CInteractionBasics.h"


//---------------------------------------------------------------------------
//!     \defgroup   collisions  Collision Detection
//---------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionBasics.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionAABB.h"
#include "collisions/CCollisionAABBBox.h"
#include "collisions/CCollisionAABBTree.h"


//---------------------------------------------------------------------------
//!     \defgroup   timers  Timers
//---------------------------------------------------------------------------
#include "timers/CFrequencyCounter.h"
#include "timers/CPrecisionClock.h"


//---------------------------------------------------------------------------
//!     \defgroup   files  Files
//---------------------------------------------------------------------------
#include "files/CFileImageBMP.h"
#include "files/CFileImageGIF.h"
#include "files/CFileImageJPG.h"
#include "files/CFileImagePNG.h"
#include "files/CFileImagePPM.h"
#include "files/CFileImageRAW.h"
#include "files/CFileModel3DS.h"
#include "files/CFileModelOBJ.h"


//---------------------------------------------------------------------------
//!     \defgroup   system  System
//---------------------------------------------------------------------------
#include "system/CGenericType.h"
#include "system/CGlobals.h"
#include "system/CMutex.h"
#include "system/CString.h"
#include "system/CThread.h"


//---------------------------------------------------------------------------
//!     \defgroup   resources Resources
//---------------------------------------------------------------------------
#include "resources/CChai3dLogo.h"
#include "resources/CFontCalibri16.h"
#include "resources/CFontCalibri18.h"
#include "resources/CFontCalibri20.h"
#include "resources/CFontCalibri22.h"
#include "resources/CFontCalibri24.h"
#include "resources/CFontCalibri26.h"
#include "resources/CFontCalibri28.h"
#include "resources/CFontCalibri32.h"
#include "resources/CFontCalibri36.h"
#include "resources/CFontCalibri40.h"
#include "resources/CFontCalibri72.h"
#include "resources/CFontCalibri144.h"


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

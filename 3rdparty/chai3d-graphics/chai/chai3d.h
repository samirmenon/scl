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
    \version   2.0.0 $Rev: 250 $

    \author    Samir Menon (modified 19 Mar 2011)
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
    <b> CHAI 3D </b> \n
    Main Header File.
*/
//===========================================================================

//---------------------------------------------------------------------------
//!     \defgroup   graphics  Graphics 
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CDraw3D.h"
#include "graphics/CGenericTexture.h"
#include "graphics/CMacrosGL.h"
#include "graphics/CMaterial.h"
#include "graphics/CTexture2D.h"
#include "graphics/CTriangle.h"
#include "graphics/CVertex.h"


//---------------------------------------------------------------------------
//!     \defgroup   math  Math 
//---------------------------------------------------------------------------
#include "math/CConstants.h"
#include "math/CMaths.h"
#include "math/CString.h"
#include "math/CMatrix3d.h"
#include "math/CQuaternion.h"
#include "math/CVector3d.h"


//---------------------------------------------------------------------------
//!     \defgroup   widgets  Widgets
//---------------------------------------------------------------------------
#include "widgets/CBitmap.h"
#include "widgets/CFont.h"
#include "widgets/CLabel.h"


//---------------------------------------------------------------------------
//!     \defgroup   scenegraph  Scenegraph
//---------------------------------------------------------------------------
#include "scenegraph/CCamera.h"
#include "scenegraph/CGenericObject.h"
#include "scenegraph/CLight.h"
#include "scenegraph/CMesh.h"
#include "scenegraph/CShapeLine.h"
#include "scenegraph/CShapeSphere.h"
#include "scenegraph/CShapeTorus.h"
#include "scenegraph/CWorld.h"


//---------------------------------------------------------------------------
//!     \defgroup   tools  Haptic Tools
//---------------------------------------------------------------------------
#include "tools/CGeneric3dofPointer.h"
#include "tools/CGenericTool.h"


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
#include "forces/CGenericPointForceAlgo.h"
#include "forces/CPotentialFieldForceAlgo.h"
#include "forces/CProxyPointForceAlgo.h"
#include "forces/CInteractionBasics.h"


//---------------------------------------------------------------------------
//!     \defgroup   collisions  Collision Detection
//---------------------------------------------------------------------------
#include "collisions/CCollisionAABB.h"
#include "collisions/CCollisionAABBBox.h"
#include "collisions/CCollisionAABBTree.h"
#include "collisions/CCollisionBasics.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionSpheres.h"
#include "collisions/CCollisionSpheresGeometry.h"
#include "collisions/CGenericCollision.h"


//---------------------------------------------------------------------------
//!     \defgroup   timers  Timers
//---------------------------------------------------------------------------
#include "timers/CPrecisionClock.h"
#include "timers/CThread.h"


//---------------------------------------------------------------------------
//!     \defgroup   files  Files
//---------------------------------------------------------------------------
#include "files/CFileLoader3DS.h"
#include "files/CFileLoaderBMP.h"
#include "files/CFileLoaderOBJ.h"
#include "files/CFileLoaderTGA.h"
#include "files/CImageLoader.h"
#include "files/CMeshLoader.h"


//---------------------------------------------------------------------------
//!     \defgroup   extras  Extras
//---------------------------------------------------------------------------
#include "extras/CGenericType.h"
#include "extras/CExtras.h"
#include "extras/CGlobals.h"


//---------------------------------------------------------------------------
//!     \defgroup   display  Viewports
//---------------------------------------------------------------------------
#if defined(_WIN32)
#include "display/CViewport.h"
#endif

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

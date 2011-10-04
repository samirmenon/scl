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
    \author    Dan Morris
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CMeshLoaderH
#define CMeshLoaderH
//---------------------------------------------------------------------------
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
#include "graphics/CVertex.h"
#include "graphics/CTriangle.h"
#include "graphics/CMaterial.h"
#include "graphics/CTexture2D.h"
#include "scenegraph/CWorld.h"
#include "scenegraph/CLight.h"
#include "scenegraph/CMesh.h"
#include "files/CFileLoaderOBJ.h"
#include "files/CFileLoader3DS.h"
#include <string>
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CMeshLoader.h 

    \brief 
    <b> Files </b> \n 
    General Mesh Loader.
*/
//===========================================================================

//---------------------------------------------------------------------------
// GLOBAL UTILITY FUNCTIONS:
//---------------------------------------------------------------------------  

/*! 
    \ingroup    files
    \brief
    Global function to load a file into a mesh.
    (CHAI currently supports .3ds and .obj files).
*/
bool cLoadMeshFromFile(cMesh* a_mesh, const string& a_fileName);

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

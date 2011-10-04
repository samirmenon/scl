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
    \version   2.0.0 $Rev: 243 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "files/CMeshLoader.h"
//--------------------------------------------------------------------------

//===========================================================================
/*!
    Global function to load a file into a mesh (CHAI currently supports
    .3ds and .obj files).  Returns true if the file is loaded successfully. \n

    The file type is determined based on the file extension supplied by
    the caller.

    \fn     bool cLoadMeshFromFile(cMesh* a_mesh, const string& a_fileName);
    \param  a_mesh  The mesh into which we should write the loaded data
    \param  a_fileName The filename from which we should load the mesh
    \return Return \b true if the file is loaded successfully, \b false 
    for an error
*/
//===========================================================================
bool cLoadMeshFromFile(cMesh* a_mesh, const string& a_fileName) 
{
    // verify mesh object
    if (a_mesh == NULL) { return (false); }

    // retrieve filename
    const char* filename = a_fileName.c_str();
    char* extension = find_extension(filename);

    // We need a file extension to figure out file type
    if (extension == 0) 
    {
        return false;
    }

    char lower_extension[1024];
    string_tolower(lower_extension,extension);

    // return value
    bool result = false;

    // Load an .obj file
    if (strcmp(lower_extension,"obj")==0) 
    {
        result = cLoadFileOBJ(a_mesh, a_fileName);
    }

    // Load a .3ds file
    else if (strcmp(lower_extension,"3ds")==0) 
    {
        result = cLoadFile3DS(a_mesh, a_fileName);
    }

    // if file has loaded, set the super parent to all child nodes.
    // the root (current object a_mesh) becomes the super parent of itself.
    if (result)
    {
        a_mesh->setSuperParent(a_mesh, true);
    }

    // return result
    return(result);
}

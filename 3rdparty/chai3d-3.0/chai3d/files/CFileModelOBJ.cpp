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
    \author    Tim Schroeder
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 322 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileModelOBJ.h"
//------------------------------------------------------------------------------
#include <iostream>
#include <iomanip>
#include <ostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <cstring>
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
bool g_objLoaderShouldGenerateExtraVertices = false;
//------------------------------------------------------------------------------


bool cLoadFileOBJ(cMultiMesh* a_object, const std::string& a_filename)
{
  cOBJModel fileObj;

  // load file into memory. If an error occurs, exit.
  if (!fileObj.LoadModel(a_filename.c_str())) { return (false); };

  return cLoadFileOBJ(a_object,fileObj, a_filename);
}

//==============================================================================
/*!
    Load an OBJ model file.

    \param  a_object  Multimesh object in which model is loaded.
    \param  a_filename  Filename.

    \return Return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileOBJ(cMultiMesh* a_object, cOBJModel &fileObj, const string& a_filename)
{
    try
    {
        // clear all vertices and triangle of current mesh
        a_object->deleteAllMeshes();	

        // get information about file
        int numMaterials = fileObj.m_OBJInfo.m_materialCount;
        int numNormals   = fileObj.m_OBJInfo.m_normalCount;
        int numTexCoord  = fileObj.m_OBJInfo.m_texCoordCount;

        // extract materials
        vector<cMaterial> materials;

        // object has no material properties
        if (numMaterials == 0)
        {
            // create a new child
            cMesh *newMesh = a_object->newMesh();

            // Give him a default color
            newMesh->setVertexColor(cColorf(1.0,1.0,1.0,1.0));
            newMesh->setUseVertexColors(true);
            newMesh->setUseMaterial(false);
            newMesh->setUseTransparency(false);
        }

        // object has material properties. Create a child for each material
        // property.
        else
        {

            int i = 0;
            bool found_transparent_material = false;

            while (i < numMaterials)
            {
                // create a new child
                cMesh *newMesh = a_object->newMesh();

                // get next material
                cMaterial newMaterial;
                cMaterialInfo material = fileObj.m_pMaterials[i];

                int textureId = material.m_textureID;
                if (textureId >= 1)
                {
                    cTexture2d *newTexture = new cTexture2d();
                    int result = newTexture->loadFromFile(material.m_texture);

                    // If this didn't work out, try again in the obj file's path
                    if (result == 0) 
                    {
                        string model_dir = cGetDirectory(a_filename);

                        char new_texture_path[1024];
                        sprintf(new_texture_path,"%s/%s",model_dir.c_str(),material.m_texture);

                        result = newTexture->loadFromFile(new_texture_path);
                    }

                    if (result)
                    {
                        newMesh->setTexture(newTexture);
                        newMesh->setUseTexture(true);
                    }
                }

                float alpha = material.m_alpha;
                if (alpha < 1.0) 
                {
                    newMesh->setUseTransparency(true, false);
                    found_transparent_material = true;
                }

                // get ambient component:
                newMesh->m_material->m_ambient.setR(material.m_ambient[0]);
                newMesh->m_material->m_ambient.setG(material.m_ambient[1]);
                newMesh->m_material->m_ambient.setB(material.m_ambient[2]);
                newMesh->m_material->m_ambient.setA(alpha);

                // get diffuse component:
                newMesh->m_material->m_diffuse.setR(material.m_diffuse[0]);
                newMesh->m_material->m_diffuse.setG(material.m_diffuse[1]);
                newMesh->m_material->m_diffuse.setB(material.m_diffuse[2]);
                newMesh->m_material->m_diffuse.setA(alpha);

                // get specular component:
                newMesh->m_material->m_specular.setR(material.m_specular[0]);
                newMesh->m_material->m_specular.setG(material.m_specular[1]);
                newMesh->m_material->m_specular.setB(material.m_specular[2]);
                newMesh->m_material->m_specular.setA(alpha);

                // get emissive component:
                newMesh->m_material->m_emission.setR(material.m_emmissive[0]);
                newMesh->m_material->m_emission.setG(material.m_emmissive[1]);
                newMesh->m_material->m_emission.setB(material.m_emmissive[2]);
                newMesh->m_material->m_emission.setA(alpha);

                // get shininess
                newMaterial.setShininess((GLuint)(material.m_shininess));

                i++;
            }

            // Enable material property rendering
            a_object->setUseVertexColors(false, true);
            a_object->setUseMaterial(true, true);

            // Mark the presence of transparency in the root mesh; don't
            // modify the value stored in children...
            a_object->setUseTransparency(found_transparent_material, false);
        }

        // Keep track of vertex mapping in each mesh; maps "old" vertices
        // to new vertices
        int nMeshes = a_object->getNumMeshes();
        vertexIndexSet_uint_map* vertexMaps = new vertexIndexSet_uint_map[nMeshes];
        vertexIndexSet_uint_map::iterator vertexMapIter;

        // build object
        {
            int i = 0;

            // get triangles
            int numTriangles = fileObj.m_OBJInfo.m_faceCount;
            int j = 0;
            while (j < numTriangles)
            {
                // get next face
                cFace face = fileObj.m_pFaces[j];

                // get material index attributed to the face
                int objIndex = face.m_materialIndex;

                // the mesh that we're reading this triangle into
                cMesh* curMesh = a_object->getMesh(objIndex);

                // create a name for this mesh if necessary (over-writing a previous
                // name if one has been written)
                if ( (face.m_groupIndex >= 0) && (fileObj.m_groupNames.size() > 0) )
                {
                    curMesh->m_name = fileObj.m_groupNames[face.m_groupIndex];
                }

                // get the vertex map for this mesh
                vertexIndexSet_uint_map* curVertexMap = &(vertexMaps[objIndex]);

                // number of vertices on face
                int vertCount = face.m_numVertices;

                if (vertCount >= 3) 
                {
                    int indexV1 = face.m_pVertexIndices[0];

                    if (g_objLoaderShouldGenerateExtraVertices==false) 
                    {
                        vertexIndexSet vis(indexV1);
                        if (numNormals  > 0) vis.nIndex = face.m_pNormalIndices[0];
                        if (numTexCoord > 0) vis.tIndex = face.m_pTextureIndices[0];
                        indexV1 = getVertexIndex(curMesh, &fileObj, curVertexMap, vis);
                    }                

                    for (int triangleVert = 2; triangleVert < vertCount; triangleVert++)
                    {
                        int indexV2 = face.m_pVertexIndices[triangleVert-1];
                        int indexV3 = face.m_pVertexIndices[triangleVert];
                        if (g_objLoaderShouldGenerateExtraVertices==false) 
                        {
                            vertexIndexSet vis(indexV2);
                            if (numNormals  > 0) vis.nIndex = face.m_pNormalIndices[triangleVert-1];
                            if (numTexCoord > 0) vis.tIndex = face.m_pTextureIndices[triangleVert-1];
                            indexV2 = getVertexIndex(curMesh, &fileObj, curVertexMap, vis);
                            vis.vIndex = indexV3;
                            if (numNormals  > 0) vis.nIndex = face.m_pNormalIndices[triangleVert];
                            if (numTexCoord > 0) vis.tIndex = face.m_pTextureIndices[triangleVert];
                            indexV3 = getVertexIndex(curMesh, &fileObj, curVertexMap, vis);
                        }                      

                        // For debugging, I want to look for degenerate triangles, but
                        // I don't want to assert here.
                        if (indexV1 == indexV2 || indexV2 == indexV3 || indexV1 == indexV3) 
                        {  
                        }

                        unsigned int indexTriangle;

                        // create triangle:
                        if (g_objLoaderShouldGenerateExtraVertices==false) 
                        {                    
                            indexTriangle =
                                curMesh->newTriangle(indexV1,indexV2,indexV3);
                        }
                        else 
                        {
                            indexTriangle =
                                curMesh->newTriangle(
                                fileObj.m_pVertices[indexV1],
                                fileObj.m_pVertices[indexV2],
                                fileObj.m_pVertices[indexV3]
                            );
                        }

                        cTriangle* curTriangle = curMesh->getTriangle(indexTriangle);

                        // assign normals:
                        if (numNormals > 0)
                        {
                            // set normals
                            curTriangle->getVertex0()->setNormal(face.m_pNormals[0]);
                            curTriangle->getVertex1()->setNormal(face.m_pNormals[triangleVert-1]);
                            curTriangle->getVertex2()->setNormal(face.m_pNormals[triangleVert]);
                        }

                        // assign texture coordinates
                        if (numTexCoord > 0)
                        {
                            // set texture coordinates
                            curTriangle->getVertex0()->setTexCoord(face.m_pTexCoords[0]);
                            curTriangle->getVertex1()->setTexCoord(face.m_pTexCoords[triangleVert-1]);
                            curTriangle->getVertex2()->setTexCoord(face.m_pTexCoords[triangleVert]);
                        }
                    }
                }
                else 
                {
                    // This faces doesn't have 3 vertices... this line is just
                    // here for debugging, since this should never happen, but
                    // I don't want to assert here.         
                }
                j++;
            }
            i++;
        }

        delete [] vertexMaps;

        // if no normals were specified in the file, compute them
        // based on triangle faces
        if (numNormals == 0) 
        {
            a_object->computeAllNormals();
        }

        // compute boundary boxes
        a_object->computeBoundaryBox(true);

        // update global position in world
        a_object->computeGlobalPositionsFromRoot(true);

        // return success
        return (true);
    }
    catch (...)
    {
        return (false);
    }
}


//==============================================================================
/*!
    Save an OBJ model file.

    \param  a_object  Multimesh object to be saved.
    \param  a_filename  Filename.

    \return Return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cSaveFileOBJ(cMultiMesh* a_object, const string& a_filename)
{
    /////////////////////////////////////////////////////////////////////////
    // INITIALIZATION
    /////////////////////////////////////////////////////////////////////////

    // get number of mesh objects composing multimesh
    unsigned int numMeshes = a_object->getNumMeshes();

    // get name of file. remove path and file extension
    string fileName = cGetFilename(a_filename, false);
    if (fileName == "") { return (false); }

    // get path
    string filePath = cGetDirectory(a_filename);


    /////////////////////////////////////////////////////////////////////////
    // MATERIAL FILE
    /////////////////////////////////////////////////////////////////////////

    // create material file
    string str = filePath + fileName + ".mat";

    // create file
    ofstream fileMat(str.c_str());

    fileMat << "#" << endl;
    fileMat << "# Wavefront material file" << endl;
    fileMat << "# CHAI3D" << endl;
    fileMat << "# http://www.chai3d.org" << endl;
    fileMat << "#" << endl;
    fileMat << endl << endl;

    // copy data from all material files
    for (unsigned int i=0; i<numMeshes; i++)
    {
        cMesh* mesh = a_object->getMesh(i);
        cMaterial* mat = mesh->m_material;

        string textureName = "";
        cTexture1d* texture = mesh->m_texture;
        if (texture != NULL)
        {
            cImage* image = texture->m_image;
            if (image != NULL)
            {
                textureName = fileName  + "-" + cStr(i) + ".png";
            }
        }

        if (mat != NULL)
        {
            fileMat << "newmtl MATERIAL"+cStr(i) << endl;
            fileMat << "Ka " << cStr(mat->m_ambient.getR(), 3) << " " << cStr(mat->m_ambient.getG(), 3) << " " << cStr(mat->m_ambient.getB(), 3) << endl;
            fileMat << "Kd " << cStr(mat->m_diffuse.getR(), 3) << " " << cStr(mat->m_diffuse.getG(), 3) << " " << cStr(mat->m_diffuse.getB(), 3) << endl;
            fileMat << "Ks " << cStr(mat->m_specular.getR(), 3) << " " << cStr(mat->m_specular.getG(), 3) << " " << cStr(mat->m_specular.getB(), 3) << endl;
            fileMat << "Ns " << cStr(mat->getShininess()) << endl;

            float transparency = mat->m_ambient.getA();
            if (transparency < 1.0)
            {
                fileMat << "d" << cStr(transparency, 3);
            }
            
            fileMat << "illum 2" << endl;
            fileMat << "map_Kd " << textureName << endl;
            fileMat << "map_bump" << endl;
            fileMat << "bump" << endl;
            fileMat << "map_opacity" << endl;
            fileMat << "map_d" << endl;
            fileMat << "refl" << endl;
            fileMat << "map_kS" << endl;
            fileMat << "map_kA" << endl;
            fileMat << "map_Ns" << endl;
            fileMat << endl;
        }
    }

    // close file
    fileMat.close();

    /////////////////////////////////////////////////////////////////////////
    // TEXTURE FILES
    /////////////////////////////////////////////////////////////////////////

    // save image files
    for (unsigned int i=0; i<numMeshes; i++)
    {
        cMesh* mesh = a_object->getMesh(i);
        cTexture1d* texture = mesh->m_texture;
        if (texture != NULL)
        {
            cImage* image = texture->m_image;
            if (image != NULL)
            {
                string textureFileName = filePath + fileName + "-" + cStr(i) + ".png";
                image->saveToFile(textureFileName);
            }
        }
    }


    /////////////////////////////////////////////////////////////////////////
    // MESH DATA
    /////////////////////////////////////////////////////////////////////////

    // vertex counter
    int vertexCounter = 0;
    int texcoordCounter = 0;

    // compute global positions for all vertices.
    a_object->computeGlobalPositions(false);

    // create OBJ file
    str = filePath + fileName + ".obj";
    ofstream fileObj(str.c_str());

    fileObj << "#" << endl;
    fileObj << "# Wavefront object file" << endl;
    fileObj << "# CHAI3D" << endl;
    fileObj << "# http://www.chai3d.org" << endl;
    fileObj << "#" << endl;
    fileObj << endl << endl;

    // basic information
    fileObj << "mtllib " << fileName << ".mat" << endl;
    fileObj << "# object " << fileName << endl;
    fileObj << "g " << fileName << endl << endl;

    // copy mesh data
    for (unsigned int i=0; i<numMeshes; i++)
    {
        // get next mesh object
        cMesh* mesh = a_object->getMesh(i);
        unsigned int numVertices = mesh->getNumVertices();

        // store vertex positions
        fileObj << "# mesh-" << cStr(i) << ": vertices" << endl;
        for (unsigned int j=0; j<numVertices; j++)
        {
            cVector3d pos = mesh->getVertex(j)->getGlobalPos();
            fileObj << "v " << cStr(pos(0), 5) << " " << cStr(pos(1), 5) << " " << cStr(pos(2), 5) << endl;
        }
        fileObj << endl;

        // store texture coordinates
        fileObj << "# mesh-" << cStr(i) << ": texture coordinates" << endl;
        for (unsigned int j=0; j<numVertices; j++)
        {
            cVector3d pos = mesh->getVertex(j)->getTexCoord();
            fileObj << "vt " << cStr(pos(0), 5) << " " << cStr(pos(1), 5) << endl;//" " << cStr(pos[2], 6) << endl;
        }
        fileObj << endl;

        // store normals
        fileObj << "# mesh-" << cStr(i) << ": normals" << endl;
        for (unsigned int j=0; j<numVertices; j++)
        {
            cVector3d normal = cMul(mesh->getLocalRot(), mesh->getVertex(j)->getNormal());
            fileObj << "vn " << cStr(normal(0), 5) << " " << cStr(normal(1), 5) << " " << cStr(normal(2), 5) << endl;
        }
        fileObj << endl;

        // faces
        fileObj << "# mesh-" << cStr(i) << ": faces" << endl;
        fileObj << "usemtl MATERIAL" << cStr(i) << endl;
        unsigned numTriangles = mesh->getNumTriangles();
        for (unsigned int j=0; j<numTriangles; j++)
        {  
            cTriangle* triangle = mesh->getTriangle(j);
            int indexV0 = triangle->getIndexVertex0() + vertexCounter + 1;
            int indexV1 = triangle->getIndexVertex1() + vertexCounter + 1;
            int indexV2 = triangle->getIndexVertex2() + vertexCounter + 1;
            int indexT0 = indexV0;
            int indexT1 = indexV1;
            int indexT2 = indexV2;
            int indexN0 = indexV0;
            int indexN1 = indexV1;
            int indexN2 = indexV2;

            fileObj << "f " << cStr(indexV0) << "/" << cStr(indexT0) << "/" << cStr(indexN0) << " " << cStr(indexV1 ) << "/" << cStr(indexT1) << "/" << cStr(indexN1) << " " << cStr(indexV2) << "/" << cStr(indexT2) << "/" << cStr(indexN2) << endl;
        }

        fileObj << endl;

        // update vertex counter
        vertexCounter = vertexCounter + mesh->getNumVertices();
    }


    // close file
    fileObj.close();

    return (true);
}


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------
//==============================================================================
// OBJ PARSER IMPLEMENTATION:
//==============================================================================

cOBJModel::cOBJModel()
{
    m_pVertices = NULL;
    m_pFaces = NULL;
    m_pNormals = NULL;
    m_pTexCoords = NULL;
    m_pMaterials = NULL;
}

//------------------------------------------------------------------------------

cOBJModel::~cOBJModel()
{
    if (m_pVertices) delete [] m_pVertices;
    if (m_pNormals) delete [] m_pNormals;
    if (m_pTexCoords) delete [] m_pTexCoords;
    if (m_pMaterials) delete [] m_pMaterials;
    if (m_pFaces)
    {
        for (unsigned int i=0; i<m_OBJInfo.m_faceCount; i++)
        {
            // Delete every pointer in the face structure
            if (m_pFaces[i].m_pNormals) delete [] m_pFaces[i].m_pNormals;
            if (m_pFaces[i].m_pNormalIndices) delete [] m_pFaces[i].m_pNormalIndices;
            if (m_pFaces[i].m_pTexCoords)  delete [] m_pFaces[i].m_pTexCoords;
            if (m_pFaces[i].m_pTextureIndices) delete [] m_pFaces[i].m_pTextureIndices;
            if (m_pFaces[i].m_pVertices)  delete [] m_pFaces[i].m_pVertices;
            if (m_pFaces[i].m_pVertexIndices)  delete [] m_pFaces[i].m_pVertexIndices;
        }
        delete [] m_pFaces;
    }
    for(unsigned int i=0; i<m_groupNames.size(); i++) {
      delete [] m_groupNames[i];
    }
}

//------------------------------------------------------------------------------

bool cOBJModel::LoadModel(const char a_fileName[])
{
    //----------------------------------------------------------------------
    // Load a OBJ file and render its data into a display list
    //----------------------------------------------------------------------

    cOBJFileInfo currentIndex;    // Current array index
    char str[C_OBJ_MAX_STR_SIZE];    // Buffer string for reading the file
    char basePath[C_OBJ_SIZE_PATH];   // Path were all paths in the OBJ start
    int nScanReturn = 0;      // Return value of fscanf
    unsigned int curMaterial = 0; // Current material

    // Get base path
    strcpy(basePath, a_fileName);
    makePath(basePath);

    //----------------------------------------------------------------------
    // Open the OBJ file
    //----------------------------------------------------------------------
    FILE *hFile = fopen(a_fileName, "r");

    // Success opening file?
    if (!hFile)
    {
        return (false);
    }

    //----------------------------------------------------------------------
    // Allocate space for structures that hold the model data
    //----------------------------------------------------------------------

    // Which data types are stored in the file ? How many of each type ?
    getFileInfo(hFile, &m_OBJInfo, basePath);

    // Vertices and faces
    if (m_pVertices) delete [] m_pVertices;
    if (m_pFaces) delete [] m_pFaces;
    m_pVertices = new cVector3d[m_OBJInfo.m_vertexCount];
    m_pFaces = new cFace[m_OBJInfo.m_faceCount];

    // Allocate space for optional model data only if present.
    if (m_pNormals) { delete [] m_pNormals; m_pNormals = NULL; }
    if (m_pTexCoords) { delete [] m_pTexCoords; m_pTexCoords = NULL; }
    if (m_pMaterials) { delete [] m_pMaterials; m_pMaterials = NULL; }
    if (m_OBJInfo.m_normalCount)
        m_pNormals = new cVector3d[m_OBJInfo.m_normalCount];
    if (m_OBJInfo.m_texCoordCount)
        m_pTexCoords = new cVector3d[m_OBJInfo.m_texCoordCount];
    if (m_OBJInfo.m_materialCount)
        m_pMaterials = new cMaterialInfo[m_OBJInfo.m_materialCount];

    // Init structure that holds the current array index
    memset(&currentIndex, 0, sizeof(cOBJFileInfo));

    //----------------------------------------------------------------------
    // Read the file contents
    //----------------------------------------------------------------------

    // Start reading the file from the start
    rewind(hFile);

    // Quit reading when end of file has been reached
    while (!feof(hFile))
    {
        // Get next string
        readNextString(str, sizeof(str), hFile);

        // Next three elements are floats of a vertex
        if (!strncmp(str, C_OBJ_VERTEX_ID, sizeof(C_OBJ_VERTEX_ID)))
        {
            // Read three floats out of the file
            float fx, fy, fz;
            nScanReturn = fscanf(hFile, "%f %f %f", &fx, &fy, &fz);

            m_pVertices[currentIndex.m_vertexCount](0)  = fx;
            m_pVertices[currentIndex.m_vertexCount](1)  = fy;
            m_pVertices[currentIndex.m_vertexCount](2)  = fz;

            // Next vertex
            currentIndex.m_vertexCount++;
        }

        // Next two elements are floats of a texture coordinate
        else if (!strncmp(str, C_OBJ_TEXCOORD_ID, sizeof(C_OBJ_TEXCOORD_ID)))
        {
            // Read two floats out of the file
            float fx, fy, fz;
            nScanReturn = fscanf(hFile, "%f %f %f", &fx, &fy, &fz);

            m_pTexCoords[currentIndex.m_texCoordCount](0)  = fx;
            m_pTexCoords[currentIndex.m_texCoordCount](1)  = fy;
            m_pTexCoords[currentIndex.m_texCoordCount](2)  = fz;

            // Next texture coordinate
            currentIndex.m_texCoordCount++;
        }

        // Next three elements are floats of a vertex normal
        else if (!strncmp(str, C_OBJ_NORMAL_ID, sizeof(C_OBJ_NORMAL_ID)))
        {
            // Read three floats out of the file
            float fx, fy, fz;
            nScanReturn = fscanf(hFile, "%f %f %f", &fx, &fy, &fz);

            m_pNormals[currentIndex.m_normalCount](0)  = fx;
            m_pNormals[currentIndex.m_normalCount](1)  = fy;
            m_pNormals[currentIndex.m_normalCount](2)  = fz;

            // Next normal
            currentIndex.m_normalCount++;
        }

        // Rest of the line contains face information
        else if (!strncmp(str, C_OBJ_FACE_ID, sizeof(C_OBJ_FACE_ID)))
        {
            // Read the rest of the line (the complete face)
            getTokenParameter(str, sizeof(str) ,hFile);

            // Convert string into a face structure
            parseFaceString(str, &m_pFaces[currentIndex.m_faceCount],
            m_pVertices, m_pNormals, m_pTexCoords, curMaterial);
            
            // Next face
            currentIndex.m_faceCount++;
        }

        // Rest of the line contains face information
        else if (!strncmp(str, C_OBJ_NAME_ID, sizeof(C_OBJ_NAME_ID)))
        {
            // Read the rest of the line (the complete face)
            getTokenParameter(str, sizeof(str) ,hFile);

            char* name = new char[strlen(str)+1];
            strcpy(name,str);
            m_groupNames.push_back(name);
        }

        // Process material information only if needed
        if (m_pMaterials)
        {
            // Rest of the line contains the name of a material
            if (!strncmp(str, C_OBJ_USE_MTL_ID, sizeof(C_OBJ_USE_MTL_ID)))
            {
                // Read the rest of the line (the complete material name)
                getTokenParameter(str, sizeof(str), hFile);
                // Are any materials loaded ?
                if (m_pMaterials)
                {
                    // Find material array index for the material name
                    for (unsigned i=0; i<m_OBJInfo.m_materialCount; i++)
                    if (!strncmp(m_pMaterials[i].m_name, str, sizeof(str)))
                    {
                        curMaterial = i;
                        break;
                    }
                }
            }

            // Rest of the line contains the filename of a material library
            else if (!strncmp(str, C_OBJ_MTL_LIB_ID, sizeof(C_OBJ_MTL_LIB_ID)))
            {
                // Read the rest of the line (the complete filename)
                getTokenParameter(str, sizeof(str), hFile);
                // Append material library filename to the model's base path
                char libraryFile[C_OBJ_SIZE_PATH];
                strcpy(libraryFile, basePath);
                strcat(libraryFile, str);

                // Append .mtl
                //strcat(szLibraryFile, ".mtl");

                // Load the material library
                loadMaterialLib(libraryFile, m_pMaterials,
                    &currentIndex.m_materialCount, basePath);
            }
        }
    }

    // close OBJ file
    fclose(hFile);

    //----------------------------------------------------------------------
    // success
    //----------------------------------------------------------------------

    return (true);
}

//------------------------------------------------------------------------------

void cOBJModel::parseFaceString(char a_faceString[], cFace *a_faceOut,
                const cVector3d *a_pVertices,
                const cVector3d *a_pNormals,
                const cVector3d *a_pTexCoords,
                const unsigned int a_materialIndex)
{
    //----------------------------------------------------------------------
    // Convert face string from the OBJ file into a face structure
    //----------------------------------------------------------------------

    unsigned int i;
    int iVertex = 0, iTextureCoord = 0, iNormal = 0;

    // Pointer to the face string. Will be incremented later to
    // advance to the next triplet in the string.
    char *pFaceString = a_faceString;

    // Save the string positions of all triplets
    int iTripletPos[C_OBJ_MAX_VERTICES];
    int iCurTriplet = 0;

    // Init the face structure
    memset(a_faceOut, 0, sizeof(cFace));

    // The first vertex always starts at position 0 in the string
    iTripletPos[0] = 0;
    a_faceOut->m_numVertices = 1;
    iCurTriplet++;
      
    if (m_groupNames.size() > 0) a_faceOut->m_groupIndex = (int)(m_groupNames.size() - 1);
    else a_faceOut->m_groupIndex = -1;

    //----------------------------------------------------------------------
    // Get number of vertices in the face
    //----------------------------------------------------------------------

    // Loop trough the whole string
    for (i=0; i<strlen(a_faceString); i++)
    {
        // Each triplet is separated by spaces
        if (a_faceString[i] == ' ')
        {
            // One more vertex
            a_faceOut->m_numVertices++;
            // Save position of triplet
            iTripletPos[iCurTriplet] = i;
            // Next triplet
            iCurTriplet++;
        }
    }

    // Face has more vertices than spaces that separate them
    // FaceOut->iNumVertices++;

    //----------------------------------------------------------------------
    // Allocate space for structures that hold the face data
    //----------------------------------------------------------------------

    // Vertices
    a_faceOut->m_pVertices = new cVector3d[a_faceOut->m_numVertices];
    a_faceOut->m_pVertexIndices = new int[a_faceOut->m_numVertices];

    // Allocate space for normals and texture coordinates only if present
    if (m_pNormals) {
        a_faceOut->m_pNormals = new cVector3d[a_faceOut->m_numVertices];
        a_faceOut->m_pNormalIndices = new int[a_faceOut->m_numVertices];
    }
    else {
        a_faceOut->m_pNormals = 0;
        a_faceOut->m_pNormalIndices = 0;
    }
        
    if (m_pTexCoords) {  
        a_faceOut->m_pTexCoords = new cVector3d[a_faceOut->m_numVertices];
        a_faceOut->m_pTextureIndices = new int[a_faceOut->m_numVertices];
    }
    else {
        a_faceOut->m_pTexCoords = 0;
        a_faceOut->m_pTextureIndices = 0;
    }

    //----------------------------------------------------------------------
    // Copy vertex, normal, material and texture data into the structure
    //----------------------------------------------------------------------

    // Set material
    a_faceOut->m_materialIndex = a_materialIndex;

    // Process per-vertex data
    for (i=0; i<(unsigned int) a_faceOut->m_numVertices; i++)
    {
        // Read one triplet from the face string

        // Are vertices, normals and texture coordinates present ?
        if (m_pNormals && m_pTexCoords)
            // Yes
            sscanf(pFaceString, "%i/%i/%i",
            &iVertex, &iTextureCoord, &iNormal);

        else if (m_pNormals && !m_pTexCoords)
            // Vertices and normals but no texture coordinates
            sscanf(pFaceString, "%i//%i", &iVertex, &iNormal);
        else if (m_pTexCoords && !m_pNormals)
            // Vertices and texture coordinates but no normals
            sscanf(pFaceString, "%i/%i", &iVertex, &iTextureCoord);
        else
            // Only vertices
            sscanf(pFaceString, "%i", &iVertex);

        // Copy vertex into the face. Also check for normals and texture
        // coordinates and copy them if present.
        memcpy(&a_faceOut->m_pVertices[i], &m_pVertices[iVertex - 1],
        sizeof(cVector3d));
        a_faceOut->m_pVertexIndices[i] = iVertex-1;

        if (m_pTexCoords) 
        {    
            memcpy(&a_faceOut->m_pTexCoords[i],
            &m_pTexCoords[iTextureCoord - 1], sizeof(cVector3d));
            a_faceOut->m_pTextureIndices[i] = iTextureCoord-1;
        }
        if (m_pNormals) 
        {
            memcpy(&a_faceOut->m_pNormals[i],
            &m_pNormals[iNormal - 1], sizeof(cVector3d));
            a_faceOut->m_pNormals[i].normalize();
            a_faceOut->m_pNormalIndices[i] = iNormal-1;
        }

        // Set string pointer to the next triplet
        pFaceString = &a_faceString[iTripletPos[i+1]];
    }
 }

//------------------------------------------------------------------------------

bool cOBJModel::loadMaterialLib(const char a_fileName[],
                cMaterialInfo* a_pMaterials,
                unsigned int* a_curMaterialIndex,
                char a_basePath[])
{
    //----------------------------------------------------------------------
    // Loads a material library file (.mtl)
    //----------------------------------------------------------------------

    char str[C_OBJ_MAX_STR_SIZE];    // Buffer used while reading the file
    bool bFirstMaterial = true;         // Only increase index after first
                                        // material has been set

    //----------------------------------------------------------------------
    // Open library file
    //----------------------------------------------------------------------

    FILE *hFile = fopen(a_fileName, "r");

    // Success ?
    if (!hFile)
    {
        return (false);
    }

    //----------------------------------------------------------------------
    // Read all material definitions
    //----------------------------------------------------------------------

    // Quit reading when end of file has been reached
    while (!feof(hFile))
    {
        int nScanReturn = 0;  // Return value of fscanf

        // Get next string
        readNextString(str, sizeof(str), hFile);

        // Is it a "new material" identifier ?
        if (!strncmp(str, C_OBJ_NEW_MTL_ID, sizeof(C_OBJ_NEW_MTL_ID)))
        {
        // Only increase index after first material has been set
        if (bFirstMaterial == true)
            // First material has been set
            bFirstMaterial = false;
        else
            // Use next index
            (*a_curMaterialIndex)++;
        // Read material name
        getTokenParameter(str, sizeof(str), hFile);
        // Store material name in the structure
        strcpy(m_pMaterials[*a_curMaterialIndex].m_name, str);
        }

        // Transparency
        if (
        (!strncmp(str, C_OBJ_MTL_ALPHA_ID, sizeof(C_OBJ_MTL_ALPHA_ID)))
        ||
        (!strncmp(str, C_OBJ_MTL_ALPHA_ID_ALT, sizeof(C_OBJ_MTL_ALPHA_ID_ALT)))
        )
        {
            // Read into current material
            nScanReturn = fscanf(hFile, "%f", &m_pMaterials[*a_curMaterialIndex].m_alpha);
        }

        // Ambient material properties
        if (!strncmp(str, C_OBJ_MTL_AMBIENT_ID, sizeof(C_OBJ_MTL_AMBIENT_ID)))
        {
            // Read into current material
            nScanReturn = fscanf(hFile, "%f %f %f",
                &m_pMaterials[*a_curMaterialIndex].m_ambient[0],
                &m_pMaterials[*a_curMaterialIndex].m_ambient[1],
                &m_pMaterials[*a_curMaterialIndex].m_ambient[2]);
        }

        // Diffuse material properties
        if (!strncmp(str, C_OBJ_MTL_DIFFUSE_ID, sizeof(C_OBJ_MTL_DIFFUSE_ID)))
        {
            // Read into current material
            nScanReturn = fscanf(hFile, "%f %f %f",
                &m_pMaterials[*a_curMaterialIndex].m_diffuse[0],
                &m_pMaterials[*a_curMaterialIndex].m_diffuse[1],
                &m_pMaterials[*a_curMaterialIndex].m_diffuse[2]);
        }

        // Specular material properties
        if (!strncmp(str, C_OBJ_MTL_SPECULAR_ID, sizeof(C_OBJ_MTL_SPECULAR_ID)))
        {
            // Read into current material
            nScanReturn = fscanf(hFile, "%f %f %f",
                &m_pMaterials[*a_curMaterialIndex].m_specular[0],
                &m_pMaterials[*a_curMaterialIndex].m_specular[1],
                &m_pMaterials[*a_curMaterialIndex].m_specular[2]);
        }

        // Texture map name
        if (!strncmp(str, C_OBJ_MTL_TEXTURE_ID, sizeof(C_OBJ_MTL_TEXTURE_ID)))
        {
            // Read texture filename
            getTokenParameter(str, sizeof(str), hFile);

            // Append material library filename to the model's base path
            char textureFile[C_OBJ_SIZE_PATH];
            strcpy(textureFile, a_basePath);
            strcat(textureFile, str);
            
            // Store texture filename in the structure
            strcpy(m_pMaterials[*a_curMaterialIndex].m_texture, textureFile);
            
            // Load texture and store its ID in the structure
            m_pMaterials[*a_curMaterialIndex].m_textureID = 1;//LoadTexture(szTextureFile);
        }

        // Shininess
        if (!strncmp(str, C_OBJ_MTL_SHININESS_ID, sizeof(C_OBJ_MTL_SHININESS_ID)))
        {
            // Read into current material
            nScanReturn = fscanf(hFile, "%f",
                &m_pMaterials[*a_curMaterialIndex].m_shininess);

            // OBJ files use a shininess from 0 to 1000; Scale for OpenGL
            m_pMaterials[*a_curMaterialIndex].m_shininess /= 1000.0f;
            m_pMaterials[*a_curMaterialIndex].m_shininess *= 128.0f;
        }
    }

    fclose(hFile);

    // Increment index cause LoadMaterialLib() assumes that the passed
    // index is always empty
    (*a_curMaterialIndex)++;

    return (true);
}

//------------------------------------------------------------------------------

void cOBJModel::getFileInfo(FILE *a_hStream, cOBJFileInfo *a_info, const char a_constBasePath[])
{
    //----------------------------------------------------------------------
    // Read the count of all important identifiers out of the given stream
    //----------------------------------------------------------------------

    char str[C_OBJ_MAX_STR_SIZE]; // Buffer for reading the file
    char basePath[C_OBJ_SIZE_PATH];  // Needed to append a filename to the base path

    // Init structure
    memset(a_info, 0, sizeof(cOBJFileInfo));

    // Rollback the stream
    rewind(a_hStream);

    // Quit reading when end of file has been reached
    while (!feof(a_hStream))
    {
        // Get next string
        readNextString(str, sizeof(str), a_hStream);

        // Vertex ?
        if (!strncmp(str, C_OBJ_VERTEX_ID, sizeof(C_OBJ_VERTEX_ID)))
        a_info->m_vertexCount++;

        // Texture coordinate ?
        if (!strncmp(str, C_OBJ_TEXCOORD_ID, sizeof(C_OBJ_TEXCOORD_ID)))
        a_info->m_texCoordCount++;
        
        // Vertex normal ?
        if (!strncmp(str, C_OBJ_NORMAL_ID, sizeof(C_OBJ_NORMAL_ID)))
        a_info->m_normalCount++;
        
        // Face ?
        if (!strncmp(str, C_OBJ_FACE_ID, sizeof(C_OBJ_FACE_ID)))
        a_info->m_faceCount++;

        // Material library definition ?
        if (!strncmp(str, C_OBJ_MTL_LIB_ID, sizeof(C_OBJ_MTL_LIB_ID)))
        {
            // Read the filename of the library
            getTokenParameter(str, sizeof(str), a_hStream);

            // Copy the model's base path into a none-constant string
            strcpy(basePath, a_constBasePath);

            // Append material library filename to the model's base path
            strcat(basePath, str);

            // Append .mtl
            //strcat(szBasePath, ".mtl");

            // Open the library file
            FILE *hMaterialLib = fopen(basePath, "r");

            // Success ?
            if (hMaterialLib)
            {
                // Quit reading when end of file has been reached
                while (!feof(hMaterialLib))
                {
                    // Read next string
                    int nScanReturn = fscanf(hMaterialLib, "%s" ,str);

                    // Is it a "new material" identifier ?
                    if (!strncmp(str, C_OBJ_NEW_MTL_ID, sizeof(C_OBJ_NEW_MTL_ID)))
                        // One more material defined
                        a_info->m_materialCount++;
                }

                // Close material library
                fclose(hMaterialLib);
            }
        }

       // Clear string two avoid counting something twice
       memset(str, '\0', sizeof(str));
    }
}

//------------------------------------------------------------------------------

void cOBJModel::readNextString(char *a_str, int a_size, FILE *a_hStream)
{
    //----------------------------------------------------------------------
    // Read the next string that isn't a comment
    //----------------------------------------------------------------------

    bool bSkipLine = false; // Skip the current line ?
    int nScanReturn = 0;  // Return value of fscanf

    // Skip all strings that contain comments
    do
    {
        // Read new string
        nScanReturn = fscanf(a_hStream, "%s", a_str);
        int len = (int)(strlen(a_str));
        if (len>0 && a_str[len-1]==13) a_str[len-1] = 0;

        // Is rest of the line a comment ?
        if (!strncmp(a_str, C_OBJ_COMMENT_ID, sizeof(C_OBJ_COMMENT_ID)))
        {
            // Skip the rest of the line
            char *ret = fgets(a_str, a_size, a_hStream);
            bSkipLine = true;
        }
        else
        {
            bSkipLine = false;
        }
    }
    while (bSkipLine == true);
}

//------------------------------------------------------------------------------

void cOBJModel::makePath(char a_fileAndPath[])
{
    //----------------------------------------------------------------------
    // Rips the filenames out of a path and adds a slash (if needed)
    //----------------------------------------------------------------------

    // Get string length
    int iNumChars = (int)(strlen(a_fileAndPath));

    // Loop from the last to the first char
    for (int iCurChar=iNumChars-1; iCurChar>=0; iCurChar--)
    {
        // If the current char is a slash / backslash
        if (a_fileAndPath[iCurChar] == char('\\') ||
        a_fileAndPath[iCurChar] == char('/'))
        {
            // Terminate the the string behind the slash / backslash
            a_fileAndPath[iCurChar + 1] = char('\0');
            return;
        }
    }

    // No slash there, set string length to zero
    a_fileAndPath[0] = char('\0');
}

//------------------------------------------------------------------------------

void cOBJModel::getTokenParameter(char a_str[],
                                  const unsigned int a_strSize, 
                                  FILE *a_hFile)
{
    char str[C_OBJ_MAX_STR_SIZE];

    //----------------------------------------------------------------------
    // Read the parameter of a token, remove space and newline character
    //----------------------------------------------------------------------

    // Read the parameter after the token
    char *ret = fgets(str, C_OBJ_MAX_STR_SIZE, a_hFile);

    // Remove newline character after the token
    int len = (int)(strlen(str));
    if (len>1 && str[len-2]==13) str[len-2] = 0;
    else                         str[len-1] = 0;

    char* first_non_whitespace_character = str;
    while( *first_non_whitespace_character == ' ' ) first_non_whitespace_character++;

    // Remove space before the token
    strcpy (a_str, first_non_whitespace_character);
}

//------------------------------------------------------------------------------

unsigned int getVertexIndex(cMesh* a_mesh, cOBJModel* a_model,
                            vertexIndexSet_uint_map* a_vertexMap, vertexIndexSet& vis) 
{
    unsigned int index;

    // Have we seen this vertex before?
    vertexIndexSet_uint_map::iterator vertexMapIter = a_vertexMap->find(vis);

    // If we have, just grab the new index for this vertex
    if (vertexMapIter != a_vertexMap->end()) 
    {
        index = (*vertexMapIter).second;
        return index;
    }

    // Otherwise create a new vertex and put the mapping in our map
    else 
    {
        index = a_mesh->newVertex(a_model->m_pVertices[vis.vIndex]);
        (*a_vertexMap)[vis] = index;
        return index;
    }
}


//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

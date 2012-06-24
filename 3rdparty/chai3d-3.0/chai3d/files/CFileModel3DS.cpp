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
    \author    Lev Povalahev
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 322 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "files/CFileModel3DS.h"
//---------------------------------------------------------------------------
#include "lib3ds.h"
#include <map>
//---------------------------------------------------------------------------
struct c3dsMaterial
{
    cMaterial* m_material;
    cTexture2d * m_texture;
    bool m_useTransparency;
    bool m_useCulling;
    bool m_useTexture;
};
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Load a 3d studio max 3ds file format image into a mesh.

    \fn         bool cLoadFile3DS(cMultiMesh* a_object, 
                  const string& a_fileName)

    \param      a_object       Object from wich 3D model file is loaded
    \param      a_fileName     Name of image file.

    \return     Return \b true if image was loaded successfully, otherwise
                return \b false.
*/
//===========================================================================
bool cLoadFile3DS(cGenericObject* a_object, 
                  const string& a_fileName)
{
    // load file
    Lib3dsFile* file = lib3ds_file_open(a_fileName.c_str());   

    // verify if file was loaded
    if (file == NULL) { return (false); }

    // vector where all material properties are stored
    vector<c3dsMaterial> matRecords;


    /////////////////////////////////////////////////////////////////////////
    // MATERIALS & TEXTURE
    /////////////////////////////////////////////////////////////////////////

    // load all material properties
    int f_nmaterials = file->nmaterials;
    Lib3dsMaterial** f_materials = file->materials;

    // parse all materials
    for (int i=0; i<f_nmaterials; i++)
    {
        // initialize variable
        c3dsMaterial matRecord;
        matRecord.m_material = NULL;
        matRecord.m_texture = NULL;
        matRecord.m_useTexture = false;
        matRecord.m_useCulling = false;
        matRecord.m_useTransparency = false;

        // get next material
        Lib3dsMaterial* f_material = *f_materials;
        f_materials++;

        // create new material
        cMaterial* material = new cMaterial();
        matRecord.m_material = material;

        // get transparency
        float transparency = 1.0 - f_material->transparency;
        if (transparency < 1.0)
        {
            matRecord.m_useTransparency = true;
        }
        else
        {
            matRecord.m_useTransparency = false;
        }

        // set ambient color
        material->m_ambient.set(f_material->ambient[0], 
                                f_material->ambient[1], 
                                f_material->ambient[2],
                                transparency);

        // set diffuse color
        material->m_diffuse.set(f_material->diffuse[0], 
                                f_material->diffuse[1], 
                                f_material->diffuse[2],
                                transparency);

        // set specular color
        material->m_specular.set(f_material->specular[0], 
                                 f_material->specular[1], 
                                 f_material->specular[2],
                                 transparency);

        // single / two sided 
        int useTwoSides = f_material->two_sided;
        if (useTwoSides == 0)
        {
            matRecord.m_useCulling = true;
        }
        else
        {
            matRecord.m_useCulling = false;
        }
   
        // get texture filename
        string name = f_material->texture1_map.name;

        // create texture if defined
        if (name != "")
        {
            // create texture object
            cTexture2d* texture = new cTexture2d();

            // load texture image
            string directory = cFindDirectory(a_fileName);
            string filename = directory + name;
            bool success = texture->loadFromFile(filename);

            // store reference to texture if loaded
            if (success)
            {
                matRecord.m_texture = texture;
                matRecord.m_useTexture = true;
            }
            else
            {
                delete texture;
                matRecord.m_texture = NULL;
                matRecord.m_useTexture = false;
            }
        }

        // store material record
        matRecords.push_back(matRecord);
    }


    /////////////////////////////////////////////////////////////////////////
    // MESHES
    /////////////////////////////////////////////////////////////////////////
    
    // get number of mesh objects
    int f_nmeshes = file->nmeshes;
    Lib3dsMesh** f_meshes = file->meshes;

    // build each mesh object
    for (int i=0; i<f_nmeshes; i++)
    {
        // get next mesh
        Lib3dsMesh* f_mesh = *f_meshes;
        f_meshes++;

        // create new CHAI3D multiMesh object
        cMultiMesh* multiMesh = NULL;
        if ((f_nmeshes == 1) && (dynamic_cast<cMultiMesh*>(a_object) != NULL))
        {
            multiMesh = (cMultiMesh*)(a_object);
        }
        else
        {
            multiMesh = new cMultiMesh();
            a_object->addChild(multiMesh);
        }


        /////////////////////////////////////////////////////////////////////////
        // COMPUTE NORMALS
        /////////////////////////////////////////////////////////////////////////
        
        // allocate table for normals
        float (*f_normals)[3] = (float(*)[3])malloc(3*3*sizeof(float)*f_mesh->nfaces);

        // compute vertex normals
        lib3ds_mesh_calculate_vertex_normals(f_mesh, f_normals);


        /////////////////////////////////////////////////////////////////////////
        // TRIANGLE DATA
        /////////////////////////////////////////////////////////////////////////
        std::map<int, cMesh*> map;

        int f_ntriangles = f_mesh->nfaces;
        
        for (int j=0; j<f_ntriangles; j++)
        {
            int index0, index1, index2;
            Lib3dsFace* f_face = &(f_mesh->faces[j]);

            // get material 
            int f_material = f_face->material;

            // get verex indices
            int f_vertex0 = f_face->index[0];
            int f_vertex1 = f_face->index[1];
            int f_vertex2 = f_face->index[2];

            // get flags
            int f_flags = f_face->flags;

            // get vertex position data
            cVector3d v0, v1, v2, n0, n1, n2, t0, t1, t2;

            if (f_mesh->vertices != NULL)
            {
                v0.set((double)(f_mesh->vertices[f_vertex0][0]),
                       (double)(f_mesh->vertices[f_vertex0][1]),
                       (double)(f_mesh->vertices[f_vertex0][2]));

                v1.set((double)(f_mesh->vertices[f_vertex1][0]),
                       (double)(f_mesh->vertices[f_vertex1][1]),
                       (double)(f_mesh->vertices[f_vertex1][2]));

                v2.set((double)(f_mesh->vertices[f_vertex2][0]),
                       (double)(f_mesh->vertices[f_vertex2][1]),
                       (double)(f_mesh->vertices[f_vertex2][2]));
            }
            else
            {
                v0.zero();
                v1.zero();
                v2.zero();
            }


            // get vertex normal data
            if (f_normals != NULL)
            {
                index0 = 3*j;
                n0.set((double)(f_normals[index0][0]),
                       (double)(f_normals[index0][1]),
                       (double)(f_normals[index0][2]));

                index1 = 3*j+1;
                n1.set((double)(f_normals[index1][0]),
                       (double)(f_normals[index1][1]),
                       (double)(f_normals[index1][2]));

                index2 = 3*j+2;
                n2.set((double)(f_normals[index2][0]),
                       (double)(f_normals[index2][1]),
                       (double)(f_normals[index2][2]));
            }
            else
            {
                n0.zero();
                n1.zero();
                n2.zero();
            }


            // get vertex texture coordinate data
            if (f_mesh->texcos != NULL)
            {
                t0.set((double)(f_mesh->texcos[f_vertex0][0]),
                       (double)(f_mesh->texcos[f_vertex0][1]),
                       0.0);

                t1.set((double)(f_mesh->texcos[f_vertex1][0]),
                       (double)(f_mesh->texcos[f_vertex1][1]),
                       0.0);

                t2.set((double)(f_mesh->texcos[f_vertex2][0]),
                       (double)(f_mesh->texcos[f_vertex2][1]),
                       0.0);
            }
            else
            {
                t0.zero();
                t1.zero();
                t2.zero();
            }

            // search for mesh
            cMesh* mesh; 
            if (map.find(f_material) == map.end())
            {
                // mesh not found. create new mesh for handling the designed material 
                mesh = multiMesh->newMesh();
                map[f_material] = mesh;

                // assign material
                mesh->setMaterial(matRecords[f_material].m_material);

                // assign texture
                if (matRecords[f_material].m_useTexture)
                {
                    mesh->setTexture(matRecords[f_material].m_texture);
                    mesh->setUseTexture(true);
                }

                // set transparency
                mesh->setUseTransparency(matRecords[f_material].m_useTransparency);
         
                // set culling
                mesh->setUseCulling(matRecords[f_material].m_useCulling);
            }
            else
            {
                mesh = map[f_material];
            }
            
            // create triangle
            mesh->newTriangle(v0, v1, v2, n0, n1, n2, t0, t1, t2);
        }

        // free normal table
        free (f_normals);
    }

    // success
    return (true);
}


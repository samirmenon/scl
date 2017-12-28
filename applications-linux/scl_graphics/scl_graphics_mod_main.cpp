/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file scl_graphics_mod_main.cpp
 *
 *  Created on: May 12, 2015
 *
 *  Copyright (C) 2015
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/scl.hpp>

#include <GL/freeglut.h>
#include <Eigen/Core>

//String tokenizer uses std::algorithm
#include <stdexcept>
#include <iostream>
#include <omp.h>
#include <algorithm>
#include <iterator>

void eigenRot2ChaiRot(const Eigen::Matrix3d& Mat, chai3d::cMatrix3d &cMat)
{
  cMat.set(
      Mat(0,0),Mat(0,1),Mat(0,2),
      Mat(1,0),Mat(1,1),Mat(1,2),
      Mat(2,0),Mat(2,1),Mat(2,2));
}

/** A sample application to modify obj graphics files. */
int main(int argc, char** argv)
{
  bool flag;
  if(argc < 3)
  {
    std::cout<<"\nscl_graphics_mod demo application modifies obj files.."
        <<"\nThe command line input is: ./<executable> <input_obj_file> <output_obj_file> <optional args>"
        <<"\n Optional args:"
        <<"\n   -s : scaling along xyz to SI units (first scale; e.g., 0.001 for mm mesh)"
        <<"\n   -t : trf = OT0 (numbers without spaces): x y z quat(x y z w) (second translate, third rotate)\n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      scl::SDatabase* db = scl::CDatabase::getData(); //Sanity Check
      if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }
      db->dir_specs_ = db->cwd_ + "../../specs/"; //Set the specs dir so scl knows where the graphics are.

      /***************************Data Structs/Classes**********************************/
      chai3d::cMultiMesh obj;    //Chai data structures for the graphics objects.
      chai3d::cMatrix3d chrot;   //Chai uses its own data structures for matrices..
      Eigen::VectorXd t;         //Temp vectors used for transforms x,y,z, quat(x,y,z,w)
      Eigen::Affine3d Trf;       //Temp matrix used for transforms
      double mesh_scaling=1;     //Scale meshes..
      t.setZero(7);

      /******************************Parse cmd args************************************/
      std::vector<std::string> argvec;
      for(int i=3;i<argc;++i)
      { argvec.push_back(std::string(argv[i])); }

      unsigned int args_ctr = 0;
      while(args_ctr < argvec.size())
      {
        if ("-t" == argvec[args_ctr])
        {//Start simulation paused
          if(args_ctr+6 >= argvec.size())
          { throw(std::runtime_error("Specified -t flag but did not specify seven subsequent args"));  }
          for(int i=0; i<7; ++i)
          { std::stringstream ss(argvec[args_ctr+1+i]);ss>>t(i); }
          std::cout<<"\n User provided t value : "<<t.transpose();
          args_ctr+=8; continue;
        }
        else if ("-s" == argvec[args_ctr])
        {// We know the next argument *should* be the scaling
          if(args_ctr+1 >= argvec.size())
          { throw(std::runtime_error("Specified -s flag but did not specify mesh scaling ratio"));  }
          std::stringstream ss(argvec[args_ctr+1]);ss>>mesh_scaling;
          std::cout<<"\n User provided mesh scaling : "<<mesh_scaling;
          args_ctr+=2; continue;
        }
        args_ctr++;
      }

      /******************************Load & Morph Obj File************************************/
      std::string f0 = argv[1],f1 = argv[2];

      chai3d::cOBJModel fileObj;
      // load file into memory. If an error occurs, exit.
      if (!fileObj.LoadModel(f0.c_str())) {
        std::cout<<"\n Could not load obj file: "<<f0;  return 1;
        return 1;
      }

      // First scale the vertices (NOTE : Important that this should convert to SI)
      for(unsigned int i=0; i<fileObj.m_OBJInfo.m_vertexCount; ++i)
      { fileObj.m_pVertices[i].mul(mesh_scaling); }

      // Now that we are in SI, we can translate the vertices
      for(unsigned int i=0; i<fileObj.m_OBJInfo.m_vertexCount; ++i)
      { fileObj.m_pVertices[i].add(t(0),t(1),t(2)); }

      // NOTE TODO : This is buggy for now! Fix it!
      // Next rotate vertices and normals
      Eigen::Quaterniond q;
      q.x() = t(3); q.y() = t(4); q.z() = t(5); q.w() = t(6);
      eigenRot2ChaiRot(q.toRotationMatrix(),chrot);

      for(unsigned int i=0; i<fileObj.m_OBJInfo.m_vertexCount; ++i)
      { chrot.mul(fileObj.m_pVertices[i]); }
      for(unsigned int i=0; i<fileObj.m_OBJInfo.m_normalCount; ++i)
      { chrot.mul(fileObj.m_pNormals[i]); }

      flag = chai3d::cLoadFileOBJ(&obj,f0);
      if(false == flag){  std::cout<<"\n Could not load obj data: "<<f0;  return 1; }

      flag = chai3d::cSaveFileOBJ(&obj,f1);
      if(false == flag){  std::cout<<"\n Could not save to obj output file: "<<f1;  return 1; }

      /****************************Deallocate Memory And Exit*****************************/
      std::cout<<"\nSCL Executed Successfully";
      std::cout<<"\n*************************\n"<<std::flush;
      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nSCL Failed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}

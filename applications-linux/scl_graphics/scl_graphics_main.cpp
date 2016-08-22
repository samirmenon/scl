/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file scl_graphics_main.cpp
 *
 *  Created on: May 11, 2015
 *
 *  Copyright (C) 2015
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>

#include <GL/freeglut.h>
#include <Eigen/Core>

//String tokenizer uses std::algorithm
#include <stdexcept>
#include <iostream>
#include <omp.h>
#include <algorithm>
#include <iterator>


bool formatTransform(const Eigen::VectorXd &t, Eigen::Affine3d &Trf)
{
  if(6!=t.rows()){  return false; }

  Trf.setIdentity();
  Eigen::Vector3d v = t.block(0,0,3,1);
  Trf.translate(v);
  Trf.rotate(Eigen::AngleAxisd(t(3), Eigen::Vector3d::UnitX()));
  Trf.rotate(Eigen::AngleAxisd(t(4), Eigen::Vector3d::UnitY()));
  Trf.rotate(Eigen::AngleAxisd(t(5), Eigen::Vector3d::UnitZ()));

  return true;
}

void eigenTrf2ChaiRot(const Eigen::Affine3d& eMat, chai3d::cMatrix3d &cMat)
{
  const Eigen::Matrix3d &Mat = eMat.rotation();
  cMat.set(
      Mat(0,0),Mat(0,1),Mat(0,2),
      Mat(1,0),Mat(1,1),Mat(1,2),
      Mat(2,0),Mat(2,1),Mat(2,2));
}

/** A sample application to view pairs of obj graphics files and fiddle with their transforms. */
int main(int argc, char** argv)
{
  bool flag;
  if(argc < 3)
  {
    std::cout<<"\nscl_graphics demo application demonstrates how to view pairs of obj files.."
        <<"\nThe command line input is: ./<executable> <obj_file0> <obj_file1> <optional args>"
        <<"\n Optional args:"
        <<"\n    -s : scaling, multiply all mesh dimensions by this number (e.g., 0.001 for mm mesh)"
        <<"\n   -t0 : trf = OT0 (numbers without spaces): x y z θx θy θz (only used at start)"
        <<"\n   -t1 : trf = 0T1 (numbers without spaces): x y z θx θy θz (used every loop iter)"
        <<"\n  -uid : The dvisor for the ui point motion (default = 5)"
        <<"\n  -fsz : The size of the origin frame. Set to zero for none. (default = 0.1m)\n";
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
      scl::SGraphicsParsed rgr;  //Robot graphics data structure...
      scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
      scl::CParserScl p;         //This time, we'll parse the tree from a file...
      Eigen::Affine3d Trf;       //Tmp var for doing transforms
      chai3d::cMatrix3d chrot;   //Chai uses its own data structures for matrices..
      chai3d::cGenericObject *cmesh0, *cmesh1; //Chai data structures for the graphics objects.
      Eigen::VectorXd t0,t1;     //Temp vectors used for transforms x,y,z, rot(x,y,z)
      double mesh_scaling=1;     //Scale meshes..
      double ui_divisor=5;       //Divide wasd motion by this.
      double frame_sz=0.1;       //The size of the origin frame

      t0.setZero(6); t1.setZero(6);

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

      flag = p.readGraphicsFromFile("./SceneCfg.xml","GrStdView",rgr);
      flag = flag && rchai.initGraphics(&rgr);
      flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
      if(false==flag) { std::cout<<"\nCouldn't1 initialize chai graphics\n"; return 1; }

      // Load the meshes...
      std::string f0 = argv[1],f1 = argv[2];
      flag = flag && rchai.addMeshToRender("mesh0",f0, Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
      flag = flag && rchai.addMeshToRender("mesh1",f1, Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());

      // Store the mesh data structures
      scl::SGraphicsChai *sgr = rchai.getChaiData();
      cmesh0 = sgr->meshes_rendered_.at("mesh0")->graphics_obj_;
      cmesh1 = sgr->meshes_rendered_.at("mesh1")->graphics_obj_;
      if(NULL == cmesh1 || NULL == cmesh0){ std::cout<<"\n Mesh not found in chai scene\n"<<std::endl;  }

      /******************************Parse cmd args************************************/
      std::vector<std::string> argvec;
      for(int i=3;i<argc;++i)
      { argvec.push_back(std::string(argv[i])); }

      unsigned int args_ctr = 0;
      while(args_ctr < argvec.size())
      {
        if ("-t0" == argvec[args_ctr])
        {//Start simulation paused
          if(args_ctr+6 >= argvec.size())
          { throw(std::runtime_error("Specified -t0 flag but did not specify six subsequent args"));  }
          for(int i=0; i<6; ++i)
          { std::stringstream ss(argvec[args_ctr+1+i]);ss>>t0(i); }
          std::cout<<"\n User provided t0 value : "<<t0.transpose();
          args_ctr+=7; continue;
        }
        else if ("-t1" == argvec[args_ctr])
        {//Start simulation paused
          if(args_ctr+6 >= argvec.size())
          { throw(std::runtime_error("Specified -t1 flag but did not specify six subsequent args"));  }
          for(int i=0; i<6; ++i)
          { std::stringstream ss(argvec[args_ctr+1+i]);ss>>t1(i); }
          scl::CDatabase::getData()->s_gui_.ui_point_[0] = ui_divisor*t1.block(0,0,3,1);
          scl::CDatabase::getData()->s_gui_.ui_point_[1] = ui_divisor*t1.block(3,0,3,1);
          std::cout<<"\n User provided t1 value : "<<t1.transpose();
          std::cout<<"\n UI points : "<<scl::CDatabase::getData()->s_gui_.ui_point_[0].transpose()<<", "
              <<scl::CDatabase::getData()->s_gui_.ui_point_[1].transpose();
          args_ctr+=7; continue;
        }
        else if ("-s" == argvec[args_ctr])
        {// We know the next argument *should* be the scaling
          if(args_ctr+1 >= argvec.size())
          { throw(std::runtime_error("Specified -s flag but did not specify mesh scaling ratio"));  }
          std::stringstream ss(argvec[args_ctr+1]);ss>>mesh_scaling;
          args_ctr+=2; continue;
        }
        else if ("-uid" == argvec[args_ctr])
        {// We know the next argument *should* be the scaling
          if(args_ctr+1 >= argvec.size())
          { throw(std::runtime_error("Specified -uid flag but did not specify ui divisor"));  }
          std::stringstream ss(argvec[args_ctr+1]);ss>>ui_divisor;
          args_ctr+=2; continue;
        }
        else if ("-fsz" == argvec[args_ctr])
        {// We know the next argument *should* be the scaling
          if(args_ctr+1 >= argvec.size())
          { throw(std::runtime_error("Specified -fsz flag but did not specify origin frame size"));  }
          std::stringstream ss(argvec[args_ctr+1]);ss>>frame_sz;
          args_ctr+=2; continue;
        }
        args_ctr++;
      }

      /***************************Parsed command line**********************************/
      std::cout<<"\n Rendering files:"
          <<"\n File 0 : "<<f0
          <<"\n Transform OT0 : "<<t0.transpose()
          <<"\n File 1 : "<<f1
          <<"\n Transform 0T1 : "<<t1.transpose();

      //Set up graphics mesh 0 (done only once)
      flag = formatTransform(t0,Trf);
      if(false == flag){  std::cout<<"\nCould not format matrix\n"; return 1; }

      cmesh0->scale(mesh_scaling,true);
      cmesh0->setFrameSize(frame_sz,true);
      cmesh0->setShowFrame(true,false);

      eigenTrf2ChaiRot(Trf,chrot);
      cmesh0->setLocalPos(t0(0),t0(1),t0(2));
      cmesh0->setLocalRot(chrot);

      //Set up graphics mesh 1 (done every loop)
      flag = formatTransform(t1,Trf);
      if(false == flag){  std::cout<<"\nCould not format matrix\n"; return 1; }

      cmesh1->scale(mesh_scaling,true);
      cmesh1->setFrameSize(frame_sz,true);
      cmesh1->setShowFrame(true,false);

      /******************************Main Loop************************************/
      std::cout<<std::flush;
      std::cout<<"\n Use wsdaeq for changing position and ikljou for orientation";
      while(true == scl::CDatabase::getData()->running_)
      {
        t1.block(0,0,3,1) = scl::CDatabase::getData()->s_gui_.ui_point_[0]/ui_divisor;
        t1.block(3,0,3,1) = scl::CDatabase::getData()->s_gui_.ui_point_[1]/ui_divisor;
        formatTransform(t1,Trf);
        eigenTrf2ChaiRot(Trf,chrot);
        cmesh1->setLocalPos(t1(0),t1(1),t1(2));
        cmesh1->setLocalRot(chrot);

        glutMainLoopEvent();
        const timespec ts = {0, 30000000};//Sleep for 30ms
        nanosleep(&ts,NULL);

        static int ii=0; ii++;
        if(0 == ii%20)
        {
          Eigen::Quaterniond q;
          q = Trf.rotation();
          q.normalize();
          std::cout<<"\n t1 : "<<t1.transpose();
          std::cout<<"\n Quaternion (xyz w): "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w();
          std::cout<<"\n Transform : \n"<<Trf.matrix()<<std::endl;
          ii = 0;
        }
      }

      /****************************Deallocate Memory And Exit*****************************/
      flag = rchai.destroyGraphics();
      if(false == flag) { throw(std::runtime_error("Error deallocating graphics pointers")); } //Sanity check.

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

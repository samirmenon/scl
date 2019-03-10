/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

This file is released under the MIT license.
See COPYING.MIT in the scl base directory.
*/
/* \file scl_advanced_quaternions.cpp
 *
 *  Created on: Mar 09, 2019
 *
 *  Copyright (C) 2019
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 *  Author: Samir Menon <smenon@cs.stanford.edu>
 */

//scl lib
#include <scl/scl.hpp>
#include <scl_ext/scl_ext.hpp>

//sutil
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>
#include <math.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

const double pi = 3.1412;

/** A sample application to demonstrate some quaternion effects.
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***********************************************\n";
  std::cout<<"Standard Control Library Tutorial (Quaternions)";
  std::cout<<"\n***********************************************\n";

  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CParserScl p;         //This time, we'll parse the tree from a file...
  sutil::CSystemClock::start();

  /******************************Simulation Params************************************/
  double transition_time=3.0; //Animation transition time (in seconds)

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).
  bool flag;

  flag = p.readGraphicsFromFile("./QuatCfg.xml","qgraphics",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let's create some objects
  chai3d::cGenericObject* obj_a = NULL, *obj_b = NULL;
  flag = rchai.addSphereToRender(Eigen::Vector3d(0,-0.5,0), obj_a, 0.07);
  flag = flag && rchai.addSphereToRender(Eigen::Vector3d(0,0.5,0), obj_b, 0.07); //Offset in space
  if(false==flag) { std::cout<<"\nCouldn't initialize spheres\n"; return 1; }

  // Show the object frames
  obj_a->setFrameSize(0.35, true);
  obj_a->setShowFrame(true);
  obj_b->setFrameSize(0.35, true);
  obj_b->setShowFrame(true);

  /*****************************Extra objects for path tracing*************************/
  const int render_path_max=1024;
  chai3d::cGenericObject * obj_path[render_path_max];
  for(int i=0; i<render_path_max; ++i)
  {
    rchai.addSphereToRender(Eigen::Vector3d::Zero(), obj_path[i], 0.01);
    obj_path[i]->setEnabled(false,false);//Disable rendering objects on chain for now.
  }

  /******************************Graphics Transforms************************************/
  // Some matrices to see what's going on
  Eigen::Affine3d T_1, T_2, T_3; // << TODO : CHANGE ME TO ADD TRANSITIONS
  Eigen::Affine3d T_base_a, T_base_b;
  Eigen::Quaterniond q_1, q_2, q_slerp;

  // Initialize a few rotations to play around with
  T_1 = Eigen::AngleAxisd(pi/6.0,Eigen::Vector3d(0,0,1));
  T_2 = Eigen::AngleAxisd(pi/6.0,Eigen::Vector3d(1,0,0));
  T_3 = Eigen::AngleAxisd(7.0*pi/6.0,Eigen::Vector3d(1,0,0));

  // Set base transformations to identity
  T_base_a.setIdentity();
  T_base_b.setIdentity();

  /******************************Graphics Rendering Loop************************************/
  // Sleep time
  const timespec ts = {0, 15000000};/*15ms*/

  // Stage the different rotation animations
  const int n_transitions = 4; // << TODO : CHANGE ME TO ADD TRANSITIONS
  double tt[n_transitions];
  tt[0] = 0.5;
  for(int x=1;x<n_transitions;x++)
  { tt[x] = tt[x-1]+transition_time;  }

  // Track the rendering path.
  int render_path_iter=0;

  // In this loop, we'll jerkily rotate one object, and smoothly rotate the other (using quaternions)
  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    glutMainLoopEvent();
    nanosleep(&ts,NULL);

    // Define the first animation (a z-axis rotation)
    if(sutil::CSystemClock::getSysTime() > tt[0] && sutil::CSystemClock::getSysTime() <= tt[1])
    {
      // Let's set one of the objects to a rotated frame
      for(static int x=0; x<1; x=1) // This is a trick to call code once
      {
        T_base_b = T_1;
        render_path_iter=0;
      }

      // And smoothly start interpolating the next one
      q_1 = T_base_a.rotation();
      q_2 = T_base_b.rotation();
      q_slerp = q_1.slerp( (sutil::CSystemClock::getSysTime() - tt[0]) / (tt[1] - tt[0]), q_2);
      obj_a->setLocalRot(q_slerp.toRotationMatrix());

      //Render path if balls still available
      if(render_path_iter < render_path_max)
      {
        chai3d::cMatrix3d Rtmp = obj_a->getGlobalRot();
        obj_path[render_path_iter]->setLocalPos(Rtmp.getCol0());
        obj_path[render_path_iter]->setEnabled(true,false);
        render_path_iter++;
      }
    }

    // Define the second animation (a multi-axis rotation)
    if(sutil::CSystemClock::getSysTime() > tt[1] && sutil::CSystemClock::getSysTime() <= tt[2])
    {
      // Let's set one of the objects to a double rotated frame
      for(static int x=0; x<1; x=1) // This is a trick to call code once
      { T_base_b = T_2 * T_1; }

      q_1 = T_base_a.rotation();
      q_2 = T_base_b.rotation();
      q_slerp = q_1.slerp( (sutil::CSystemClock::getSysTime() - tt[1]) / (tt[2] - tt[1]), q_2);
      obj_a->setLocalRot(q_slerp.toRotationMatrix());

      //Render path if balls still available
      if(render_path_iter < render_path_max)
      {
        chai3d::cMatrix3d Rtmp = obj_a->getGlobalRot();
        obj_path[render_path_iter]->setLocalPos(Rtmp.getCol0());
        obj_path[render_path_iter]->setEnabled(true,false);
        render_path_iter++;
      }
    }

    // Define the third animation (a multi-axis rotation)
    if(sutil::CSystemClock::getSysTime() > tt[2] && sutil::CSystemClock::getSysTime() <= tt[3])
    {
      // Let's set one of the objects to a double rotated frame
      for(static int x=0; x<1; x=1) // This is a trick to call code once
      { T_base_b = T_3 * T_1; }

      q_1 = T_base_a.rotation();
      q_2 = T_base_b.rotation();
      q_slerp = q_1.slerp( (sutil::CSystemClock::getSysTime() - tt[2]) / (tt[3] - tt[2]), q_2);
      obj_a->setLocalRot(q_slerp.toRotationMatrix());

      //Render path if balls still available
      if(render_path_iter < render_path_max)
      {
        chai3d::cMatrix3d Rtmp = obj_a->getGlobalRot();
        obj_path[render_path_iter]->setLocalPos(Rtmp.getCol0());
        obj_path[render_path_iter]->setEnabled(true,false);
        render_path_iter++;
      }
    }

    // << TODO : ADD ONE MORE IF (copy/paste/modify from above) TO ADD TRANSITIONS

    obj_b->setLocalRot(T_base_b.rotation());
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n Rendered path using n-via-points = "<<render_path_iter;
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}

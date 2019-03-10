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
  flag = rchai.addSphereToRender(Eigen::Vector3d::Zero(), obj_a, 0.07);
  flag = flag && rchai.addSphereToRender(Eigen::Vector3d(0,0.5,0), obj_b, 0.07); //Offset in space
  if(false==flag) { std::cout<<"\nCouldn't initialize spheres\n"; return 1; }

  // Show the object frames
  obj_a->setFrameSize(0.25, true);
  obj_a->setShowFrame(true);
  obj_b->setFrameSize(0.25, true);
  obj_b->setShowFrame(true);

  /******************************Graphics Rendering************************************/
  // Some matrices to see what's going on
  Eigen::Affine3d T_1, T_2, T_base_a, T_base_b;
  Eigen::Quaterniond q_1, q_2, q_slerp;

  // Initialize a few rotations to play around with
  // Rotate << cos(pi/6), -sin(pi/6), 0, sin(pi/6), cos(pi/6), 0, 0, 0, 1;
  T_1 = Eigen::AngleAxisd(pi/6,Eigen::Vector3d(0,0,1));
  // Rotate << 1, 0, 0, 0, cos(pi/6), -sin(pi/6), 0, sin(pi/6), cos(pi/6);
  T_2 = Eigen::AngleAxisd(pi/6,Eigen::Vector3d(1,0,0));

  // Set base transformations to identity
  T_base_a.setIdentity();
  T_base_b.setIdentity();

  // In this loop, we'll jerkily rotate one object, and smoothly rotate the other (using quaternions)
  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    glutMainLoopEvent();
    const timespec ts = {0, 15000000};/*15ms*/
    nanosleep(&ts,NULL);

    const double t_next = 3.0, t_next2 = 5.0, t_stasis = 7.0;

    if(sutil::CSystemClock::getSysTime() > t_next && sutil::CSystemClock::getSysTime() <= t_next2)
    {
      // Let's set one of the objects to a rotated frame
      for(static int x=0; x<1; x=1) // This is a trick to call code once
      { T_base_b = T_1;  }

      // And smoothly start interpolating the next one
      q_1 = T_base_a.rotation();
      q_2 = T_base_b.rotation();
      q_slerp = q_1.slerp( (sutil::CSystemClock::getSysTime() - t_next) / (t_next2 - t_next), q_2);
      obj_a->setLocalRot(q_slerp.toRotationMatrix());
    }

    if(sutil::CSystemClock::getSysTime() > t_next2 && sutil::CSystemClock::getSysTime() <= t_stasis)
    {
      // Let's set one of the objects to a double rotated frame
      for(static int x=0; x<1; x=1) // This is a trick to call code once
      { T_base_b = T_2 * T_1; }

      q_1 = T_base_a.rotation();
      q_2 = T_base_b.rotation();
      q_slerp = q_1.slerp( (sutil::CSystemClock::getSysTime() - t_next2) / (t_stasis - t_next2), q_2);
      obj_a->setLocalRot(q_slerp.toRotationMatrix());
    }

    obj_b->setLocalRot(T_base_b.rotation());
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}

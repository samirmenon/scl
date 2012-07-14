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
/* \file ChaiGlutHandlers.cpp
 *
 *  Created on: Oct 27, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

/* Handler functions for chai.
 *
 * NOTE : Right now, scl only supports a single glut display
 *        instance. It renders the first chai graphics instance
 *        by default.
 */

#include <scl/graphics/chai/ChaiGlutHandlers.hpp>

#include <GL/freeglut.h>

namespace scl_chai_glut_interface
{
  //---------------------------------------------------------------------------
  SChaiGlobals::SChaiGlobals()
  {
    for(int i=0; i<256; ++i)
      keys_active[i] = false;
    GLOB_chaiDbptr = S_NULL;
    cam_sph_x_=3.0;
    cam_sph_h_= 0.0;
    cam_sph_v_=0.0;
    chai_glut = S_NULL;
    chai_glut_running=true;
  }

  /**
   * Picks the passed chai definition (by string name) on the pile
   */
  bool initializeGlutForChai(const std::string & arg_graphics_name,
      scl::CChaiGraphics *arg_chai_glut)
  {
    try
    {
      scl::SDatabase *db = scl::CDatabase::getData();
      if(S_NULL == db)
      { throw(std::runtime_error("Database not intialized")); }

      if(S_NULL == arg_chai_glut)
      { throw(std::runtime_error("Passed invalid chai object")); }
      CChaiGlobals::getData()->chai_glut = arg_chai_glut;

      CChaiGlobals::getData()->GLOB_chaiDbptr = db->s_gui_.chai_data_.at(arg_graphics_name);

      if(S_NULL == CChaiGlobals::getData()->GLOB_chaiDbptr)
      { throw(std::runtime_error("Couldn't find the specified graphics instance")); }

      // retrieve the resolution of the computer display and estimate the position
      // of the GLUT window so that it is located at the center of the screen
      int screenW = glutGet(GLUT_SCREEN_WIDTH);
      int screenH = glutGet(GLUT_SCREEN_HEIGHT);

      CChaiGlobals::getData()->GLOB_windowPosX = (screenW - CChaiGlobals::getData()->GLOB_chaiDbptr->gl_width_) / 2;
      CChaiGlobals::getData()->GLOB_windowPosY = (screenH - CChaiGlobals::getData()->GLOB_chaiDbptr->gl_height_) / 2;

      CChaiGlobals::getData()->GLOB_chaiDbptr->running_ = true;

      // initialize the OpenGL GLUT window
      glutInitWindowPosition(scl_chai_glut_interface::CChaiGlobals::getData()->GLOB_windowPosX,
          scl_chai_glut_interface::CChaiGlobals::getData()->GLOB_windowPosX);
      glutInitWindowSize(scl_chai_glut_interface::CChaiGlobals::getData()->GLOB_chaiDbptr->gl_width_,
          scl_chai_glut_interface::CChaiGlobals::getData()->GLOB_chaiDbptr->gl_height_);
      glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
      glutCreateWindow("scl_busylizzy");

      //Set up glut's handlers
      glutDisplayFunc(scl_chai_glut_interface::updateGraphics);
      glutKeyboardFunc(scl_chai_glut_interface::keyPressed);
      glutKeyboardUpFunc(scl_chai_glut_interface::keyReleased);
      glutReshapeFunc(scl_chai_glut_interface::resizeWindow);
      glutMouseFunc(mouseClick);
      glutMotionFunc(mouseMove);
      glutSetWindowTitle("SCL scl (v0.2)");

      // create a mouse menu (right button)
      glutCreateMenu(scl_chai_glut_interface::menuSelect);
      glutAddMenuEntry("full screen",
          scl_chai_glut_interface::SChaiGlobals::OPTION_FULLSCREEN);
      glutAddMenuEntry("window display",
          scl_chai_glut_interface::SChaiGlobals::OPTION_WINDOWDISPLAY);
      glutAddMenuEntry("toggle mouse cam",
          scl_chai_glut_interface::SChaiGlobals::OPTION_TOGGLE_MOUSE_CAM_SELECT);
      glutAttachMenu(GLUT_MIDDLE_BUTTON);
    }
    catch(std::exception & e)
    {
      std::cout<<"\ninitializeGlutForChai() Error: "<< e.what();
      return false;
    }
    return true;
  }

  //---------------------------------------------------------------------------

  void resizeWindow(int w, int h)
  {
    // update the size of the viewport
    CChaiGlobals::getData()->GLOB_chaiDbptr->gl_width_ = w;
    CChaiGlobals::getData()->GLOB_chaiDbptr->gl_height_ = h;
    glViewport(0, 0,
        CChaiGlobals::getData()->GLOB_chaiDbptr->gl_width_,
        CChaiGlobals::getData()->GLOB_chaiDbptr->gl_height_);
  }

  //---------------------------------------------------------------------------
  // callback when a keyboard key is pressed
  void keyPressed(unsigned char key, int x, int y)
  {
    CChaiGlobals::getData()->keys_active[key] = true;
  }

  // callback when a keyboard key is released
  void keyReleased(unsigned char key, int x, int y)
  {
    CChaiGlobals::getData()->keys_active[key] = false;
  }


  void keyHandler()
  {
    // escape key
    if ((CChaiGlobals::getData()->keys_active[27]) || (CChaiGlobals::getData()->keys_active[static_cast<int>('x')]))
    {
      // close everything
      closeChaiGlut();
      // set running flag to false
      CChaiGlobals::getData()->chai_glut_running = false;
    }

    // option 1: Enable/Disable textures
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('1')])
    {
      bool useTexture = CChaiGlobals::getData()->GLOB_chaiDbptr->chai_world_->getUseTexture();
      CChaiGlobals::getData()->GLOB_chaiDbptr->chai_world_->setUseTexture(!useTexture,true);
    }

    // option 2: Enable/Disable wireframe mode
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('2')])
    {
      bool useWireMode = CChaiGlobals::getData()->GLOB_chaiDbptr->chai_world_->getWireMode();
      CChaiGlobals::getData()->GLOB_chaiDbptr->chai_world_->setWireMode(!useWireMode, true);
    }

    if (CChaiGlobals::getData()->keys_active[static_cast<int>('9')])
    { scl::CDatabase::getData()->s_gui_.ui_point_selector_1_++; }
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('0')])
    { scl::CDatabase::getData()->s_gui_.ui_point_selector_1_ = 0; }

    /** Control the gui 3D point */
    const scl::sFloat opt_mult=1;
    if(scl::CDatabase::getData()->s_gui_.ui_point_selector_1_ == 0)
    {
      //Op point 1 : X - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('s')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) += opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('s')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('S')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) += opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('S')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('w')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) -= opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('w')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('W')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) -= opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('W')] = false;
      }

      //Op point 1 : Y - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('d')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) += opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('d')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('D')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) += opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('D')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('a')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) -= opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('W')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('A')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) -= opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('W')] = false;
      }


      //Op point 1 : Z - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('e')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) += opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('e')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('E')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) += opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('E')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('q')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) -= opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('q')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('Q')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) -= opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('Q')] = false;
      }

      //Op point 2 : X - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('j')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) += opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('j')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('J')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) += opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('J')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('u')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) -= opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('u')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('U')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) -= opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('U')] = false;
      }

      //Op point 2 : Y - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('k')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) += opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('k')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('K')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) += opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('K')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('h')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) -= opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('h')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('H')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) -= opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('H')] = false;
      }

      //Op point 2 : Z - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('i')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) += opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('i')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('I')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) += opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('I')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('y')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) -= opt_mult*0.01;
        CChaiGlobals::getData()->keys_active[static_cast<int>('y')] = false;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('Y')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) -= opt_mult*0.05;
        CChaiGlobals::getData()->keys_active[static_cast<int>('y')] = false;
      }
    }
    else
    {
      //Op point 1 : X - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('s')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) += opt_mult*0.01;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) += opt_mult*0.01;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('S')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) += opt_mult*0.05;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) += opt_mult*0.05;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('w')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) -= opt_mult*0.01;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) -= opt_mult*0.01;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('W')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](0) -= opt_mult*0.05;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](0) -= opt_mult*0.05;
      }

      //Op point 1 : Y - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('d')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) += opt_mult*0.01;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) += opt_mult*0.01;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('D')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) += opt_mult*0.05;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) += opt_mult*0.05;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('a')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) -= opt_mult*0.01;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) -= opt_mult*0.01;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('A')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](1) -= opt_mult*0.05;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](1) -= opt_mult*0.05;
      }

      //Op point 1 : Z - axis
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('e')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) += opt_mult*0.01;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) += opt_mult*0.01;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('E')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) += opt_mult*0.05;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) += opt_mult*0.05;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('q')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) -= opt_mult*0.01;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) -= opt_mult*0.01;
      }
      if (CChaiGlobals::getData()->keys_active[static_cast<int>('Q')])
      {
        scl::CDatabase::getData()->s_gui_.ui_point_[0](2) -= opt_mult*0.05;
        scl::CDatabase::getData()->s_gui_.ui_point_[1](2) -= opt_mult*0.05;
      }
    }

    //Print debug info:
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('b')])
    {
      std::cout<<"\nOperational Point Pos : "<<
          scl::CDatabase::getData()->s_gui_.ui_point_[0].transpose();
      std::cout<<"\nOperational Point 2 Pos : "<<
          scl::CDatabase::getData()->s_gui_.ui_point_[1].transpose();
    }

    //Pause the controller
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('p')])
    {
      scl::CDatabase::getData()->pause_ctrl_dyn_ = true;
      std::cout<<"\nController and Simulation Paused. Press 'P' to unpause or ';' to step. "<<std::flush;
      CChaiGlobals::getData()->keys_active[static_cast<int>('p')] = false;
    }

    //Unpause the controller
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('P')])
    {
      scl::CDatabase::getData()->pause_ctrl_dyn_ = false;
      std::cout<<"\nController and Simulation Unpaused.";
      CChaiGlobals::getData()->keys_active[static_cast<int>('P')] = false;
    }

    //Step the controller
    if (CChaiGlobals::getData()->keys_active[static_cast<int>(';')])
    {
      scl::CDatabase::getData()->step_ctrl_dyn_ = true;
      std::cout<<"+1"<<std::flush;
    }

    //Toggle logging
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('/')])
    {
      scl::CDatabase::getData()->param_logging_on_ = ~scl::CDatabase::getData()->param_logging_on_;
      if(scl::CDatabase::getData()->param_logging_on_)
      { std::cout<<"\nStarting logging"<<std::flush;  }
      else
      { std::cout<<"\nStopping logging"<<std::flush;  }
      CChaiGlobals::getData()->keys_active[static_cast<int>('/')] = false;
    }

    //Modulate a robot's links
    static int link_idx = 0;
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('+')])
    {
      link_idx++;

      scl::SDatabase* db = scl::CDatabase::getData();
      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
      if(link_idx >= io->sensors_.q_.size())
      { link_idx = io->sensors_.q_.size()-1;  }

      std::cout<<"\n"<<io->name_<<" : Link : "<<link_idx<<std::flush;
    }
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('-')])
    {
      link_idx--;
      if(0>link_idx) link_idx = 0;

      scl::SDatabase* db = scl::CDatabase::getData();
      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);

      std::cout<<"\n"<<io->name_<<" : Link : "<<link_idx<<std::flush;
    }

    // Position keyboard shortcuts.
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('o')])
    {
      scl::SDatabase* db = scl::CDatabase::getData();
      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
      if(link_idx >= io->sensors_.q_.size())
      { link_idx = io->sensors_.q_.size()-1;  }
      io->sensors_.q_(link_idx) += 0.1;
    }
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('O')])
    {
      scl::SDatabase* db = scl::CDatabase::getData();
      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
      if(link_idx >= io->sensors_.q_.size())
      { link_idx = io->sensors_.q_.size()-1;  }
      io->sensors_.q_(link_idx) += 0.3;
    }

    if (CChaiGlobals::getData()->keys_active[static_cast<int>('l')])
    {
      scl::SDatabase* db = scl::CDatabase::getData();
      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
      if(link_idx >= io->sensors_.q_.size())
      { link_idx = io->sensors_.q_.size()-1;  }
      io->sensors_.q_(link_idx) -= 0.1;
    }
    if (CChaiGlobals::getData()->keys_active[static_cast<int>('L')])
    {
      scl::SDatabase* db = scl::CDatabase::getData();
      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
      if(link_idx >= io->sensors_.q_.size())
      { link_idx = io->sensors_.q_.size()-1;  }
      io->sensors_.q_(link_idx) -= 0.3;
    }

//    //Velocity and acceleration keyboard shortcuts.
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('o')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.dq_.size())
//      { link_idx = io->sensors_.dq_.size()-1;  }
//      io->sensors_.dq_(link_idx) += 0.1;
//    }
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('O')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.dq_.size())
//      { link_idx = io->sensors_.dq_.size()-1;  }
//      io->sensors_.dq_(link_idx) += 0.3;
//    }
//
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('k')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.dq_.size())
//      { link_idx = io->sensors_.dq_.size()-1;  }
//      io->sensors_.dq_(link_idx) -= 0.1;
//    }
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('K')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.dq_.size())
//      { link_idx = io->sensors_.dq_.size()-1;  }
//      io->sensors_.dq_(link_idx) -= 0.3;
//    }
//
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('i')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.ddq_.size())
//      { link_idx = io->sensors_.ddq_.size()-1;  }
//      io->sensors_.ddq_(link_idx) += 10;
//    }
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('I')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.ddq_.size())
//      { link_idx = io->sensors_.ddq_.size()-1;  }
//      io->sensors_.ddq_(link_idx) += 50;
//    }
//
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('j')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.ddq_.size())
//      { link_idx = io->sensors_.ddq_.size()-1;  }
//      io->sensors_.ddq_(link_idx) -= 10;
//    }
//    if (CChaiGlobals::getData()->keys_active[static_cast<int>('J')])
//    {
//      scl::SDatabase* db = scl::CDatabase::getData();
//      db->s_io_.io_data_.resetIterator();
//      scl::SRobotIOData* io = db->s_io_.io_data_.at(0);
//      if(link_idx >= io->sensors_.ddq_.size())
//      { link_idx = io->sensors_.ddq_.size()-1;  }
//      io->sensors_.ddq_(link_idx) -= 50;
//    }
  }

  //---------------------------------------------------------------------------

  void menuSelect(int value)
  {
    switch (value)
    {
      // enable full screen display
      case SChaiGlobals::OPTION_FULLSCREEN:
        glutFullScreen();
        break;

        // reshape window to original size
      case SChaiGlobals::OPTION_WINDOWDISPLAY:
        glutReshapeWindow(CChaiGlobals::getData()->GLOB_chaiDbptr->gl_width_
            , CChaiGlobals::getData()->GLOB_chaiDbptr->gl_height_);
        break;

        //Whether click-drag rotates the camera or adds a force.
      case SChaiGlobals::OPTION_TOGGLE_MOUSE_CAM_SELECT:
        CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_mode_cam_ = ~(CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_mode_cam_);
        break;
    }
  }

  //---------------------------------------------------------------------------
  // stop the simulation
  void closeChaiGlut(void)
  { CChaiGlobals::getData()->GLOB_chaiDbptr->running_ = false;  }

  //---------------------------------------------------------------------------
  void updateGraphics(void)
  {
    //Update the IO
    keyHandler();

#ifdef W_TESTING
    bool flag;
    flag = CChaiGlobals::getData()->chai_glut->updateGraphics();
    if (false == flag) { printf("\nChai-Glut Error: Could not update graphics successfully.");  }
#else
    //NOTE : No checks in release mode
    CChaiGlobals::getData()->chai_glut->updateGraphics();
#endif

    /**
     * NOTE TODO : Implement showing a clock on the display
     *
    std::stringstream ss;
    ss << scl::CSystemClock::get_clock()->get_sim_time();
    std::string tmp;
    ss >> tmp;
    glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char*) tmp.c_str());
     */

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
#ifdef W_TESTING
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR)
    { printf("Error:  %s\n", gluErrorString(err));  }
#endif

    // inform the GLUT window to call updateGraphics again (next frame)
    if (CChaiGlobals::getData()->GLOB_chaiDbptr->running_)
    { glutPostRedisplay();  }
  }

  //---------------------------------------------------------------------------

  //---------------------------------------------------------------------------

  void mouseClick(int button, int state, int x, int y)
  {
    // mouse button down
    if (state == GLUT_DOWN)
    {
      CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_button_pressed_ = true;
      CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_x_ = x;
      CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_y_ = y;
      CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_button_ = button;
    }

    // mouse button up
    else if (state == GLUT_UP)
    {
      CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_button_pressed_ = false;
    }
  }

  //---------------------------------------------------------------------------

  void mouseMove(int x, int y)
  {
    if(true==CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_mode_cam_)
    {
      if(CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_button_pressed_)
      {
        if (CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_button_ == GLUT_RIGHT_BUTTON)
        { CChaiGlobals::getData()->cam_sph_x_ = CChaiGlobals::getData()->cam_sph_x_ - 0.01 * (y - CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_y_);  }

        else if (CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_button_ == GLUT_LEFT_BUTTON)
        {
          CChaiGlobals::getData()->cam_sph_h_ = CChaiGlobals::getData()->cam_sph_h_ - (x - CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_x_);
          CChaiGlobals::getData()->cam_sph_v_ = CChaiGlobals::getData()->cam_sph_v_ + (y - CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_y_);
        }
        updateCameraPosition();
      }
    }
    else
    {//Attach a force

    }

    CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_x_ = x;
    CChaiGlobals::getData()->GLOB_chaiDbptr->mouse_y_ = y;
  }

  //---------------------------------------------------------------------------

  void updateCameraPosition()
  {
    // check values
    static scl::SGraphicsParsedData* gr_ds = scl::CDatabase::getData()->s_parser_.graphics_worlds_.at(CChaiGlobals::getData()->GLOB_chaiDbptr->name_);

    if (CChaiGlobals::getData()->cam_sph_x_ < 0.1) { CChaiGlobals::getData()->cam_sph_x_ = 0.1; }
    if (CChaiGlobals::getData()->cam_sph_v_ > 89) { CChaiGlobals::getData()->cam_sph_v_ = 89; }
    if (CChaiGlobals::getData()->cam_sph_v_ < -89) { CChaiGlobals::getData()->cam_sph_v_ = -89; }

    // compute position of camera in space
    cVector3d pos = cAdd(
        gr_ds->cam_lookat_,
        cVector3d(
            CChaiGlobals::getData()->cam_sph_x_ * cCosDeg(CChaiGlobals::getData()->cam_sph_h_) * cCosDeg(CChaiGlobals::getData()->cam_sph_v_),
            CChaiGlobals::getData()->cam_sph_x_ * cSinDeg(CChaiGlobals::getData()->cam_sph_h_) * cCosDeg(CChaiGlobals::getData()->cam_sph_v_),
            CChaiGlobals::getData()->cam_sph_x_ * cSinDeg(CChaiGlobals::getData()->cam_sph_v_)
        )
    );

    // set new position to camera
    CChaiGlobals::getData()->GLOB_chaiDbptr->chai_cam_->set(pos,gr_ds->cam_lookat_,gr_ds->cam_up_);

    // recompute global positions
    CChaiGlobals::getData()->GLOB_chaiDbptr->chai_world_->computeGlobalPositions(true);
  }

}

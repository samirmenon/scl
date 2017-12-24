/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file ChaiGlutHandlers.hpp
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

#ifndef CHAIGLUTHANDLERS_HPP_
#define CHAIGLUTHANDLERS_HPP_

#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <sutil/CSingleton.hpp>

#include "chai3d.h"
#include <scl/graphics/chai/CGraphicsChai.hpp>

#include <string>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace scl_chai_glut_interface
{
  //---------------------------------------------------------------------------
  // DECLARED GLOBALS & DATA
  //---------------------------------------------------------------------------
  class SChaiGlobals
  {
  public:
    static const scl::sUInt OPTION_FULLSCREEN=0;
    static const scl::sUInt OPTION_WINDOWDISPLAY=1;
    static const scl::sUInt OPTION_TOGGLE_MOUSE_CAM_SELECT=2;
    static const scl::sUInt OPTION_TOGGLE_MOUSE_MOVE_SCENE=3;

    scl::SGraphicsChai* GLOB_chaiDbptr; //GLOB == Global var.
    scl::SGraphicsParsed* GLOB_gr_parsed_ds;

    //Window information (and thus not stored in the
    //graphics--unlike the GL width and height)
    scl::sInt GLOB_windowPosX;
    scl::sInt GLOB_windowPosY;

    /** Spherical coordinates for the camera.
     * Position, horizontal angle, vertical angle */
    scl::sFloat cam_sph_x_,cam_sph_h_,cam_sph_v_;

    /** Dynamic positioning for the camera lookat */
    scl::sFloat cam_lookat_x_, cam_lookat_y_, cam_lookat_z_;

    scl::CGraphicsChai *chai_glut;

    scl::sBool chai_glut_running;

    scl::sBool keys_active[256];

    /** Constructor sets everything to zero / false */
    SChaiGlobals();
  };

  /** A singleton to store the chai global variables. */
  typedef sutil::CSingleton<SChaiGlobals> CChaiGlobals;

  //---------------------------------------------------------------------------
  // DECLARED FUNCTIONS
  //---------------------------------------------------------------------------
  bool initializeGlutForChai(const std::string & arg_graphics_name,
      scl::CGraphicsChai *arg_chai_glut);

  // Initializes glut from a data structure instead of the databse
  bool initializeGlutForChai(scl::SGraphicsParsed* arg_gr_parsed_ds,
      scl::CGraphicsChai *arg_chai_glut);

  // callback when the window display is resized
  void resizeWindow(int w, int h);

  // callback when a keyboard key is pressed
  void keyPressed(unsigned char key, int x, int y);

  // callback when a keyboard key is released
  void keyReleased(unsigned char key, int x, int y);

  // callback when a keyboard key is pressed
  void keyHandler();

  // callback when the right mouse button is pressed to select a menu item
  void menuSelect(int value);

  // function called before exiting the application
  void closeChaiGlut(void);

  // main graphics callback
  void updateGraphics(void);

  // callback to handle mouse click
  void mouseClick(int button, int state, int x, int y);

  // callback to handle mouse motion
  void mouseMove(int x, int y);

  // callback to handle mouse motion
  void mousePassiveMove(int x, int y);

  // function to update the viewing screen
  void updateCameraPosition();
}

#endif /* CHAIGLUTHANDLERS_HPP_ */

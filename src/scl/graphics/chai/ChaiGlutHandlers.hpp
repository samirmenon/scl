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

    scl::SGraphicsChai* GLOB_chaiDbptr;

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

  // function to update the viewing screen
  void updateCameraPosition();
}

#endif /* CHAIGLUTHANDLERS_HPP_ */

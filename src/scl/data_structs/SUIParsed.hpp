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
/* \file SUIParsed.hpp
 *
 *  Created on: Apr 28, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SUIPARSED_HPP_
#define SUIPARSED_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>

#include <Eigen/Core>

#include <vector>

namespace scl
{

  /**
   * Contains all the static information for user interface settings.
   *
   * Set input devices, ui-point attachments etc. here.
   *
   * This class is intended to be read by an app, which can use the information
   * to set up its interface.
   *
   * Purpose : Provide spec => Can be used to attach ui objects to controllers etc.
   */
  class SUIParsed : public SObject
  {
  public:
    /** An enum to specify the keyboard set used */
    enum EKbdKeySpec {
      KBD_KEY_swdaeq=0, KBD_KEY_kiljou=1, KBD_KEY_gthfyr=2,
      KBD_KEY_WSDAEQ=3, KBD_KEY_KILJOU=4, KBD_KEY_GTHFYR=5,
    };

    /** Recommend a task->keyboard pairing */
    class SUIKeyboardControl { public: EKbdKeySpec idx; std::string name_, robot_name_, task_name_; };

    /** Recommend a task->keyboard pairing */
    class SUIHapticControl { public: std::string name_, haptic_device_name_, robot_name_, task_name_; };

    /** Holds the information about the keyboard ui points from the xml file */
    sutil::CMappedList<std::string, SUIKeyboardControl> ui_kbd_points_;

    /** Holds the information about how to connect any attached haptic devices to a robot */
    sutil::CMappedList<std::string, SUIHapticControl> ui_haptic_points_;

    /** Constructor : Sets stuff to zero. */
    SUIParsed() : SObject(std::string("SUIParsedParsed")){}
  };

}

#endif /* SUIPARSED_HPP_ */

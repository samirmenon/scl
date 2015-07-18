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
/* \file SGraphicsParsed.hpp
 *
 *  Created on: Sep 5, 2010
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SGRAPHICSPARSED_HPP_
#define SGRAPHICSPARSED_HPP_

#include <vector>
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>

#include <Eigen/Core>

namespace scl
{
  /**
   * Contains all the static information for rendering a scenegraph.
   *
   * Set cameras, lights etc. here.
   */
  class SGraphicsParsed : public SObject
  {
  public:
    /** To position and orient a light */
    class SLight
    {
    public:
      sFloat pos_[3];///The light's position.
      sFloat lookat_[3];///Point the light in a direction.
    };

    /** Camera settings for this world. */
    Eigen::Vector3d cam_pos_;///The camera's position.
    Eigen::Vector3d cam_lookat_;///Position the camera looks at.
    Eigen::Vector3d  cam_up_;///The direction of the up-vector.
    sFloat cam_clipping_dist_[2];///Defines rendering limits along cam's line of sight (meters)

    sFloat background_color_[3];///RGB.

    std::string file_background_;///Could provide a 2d image to be used as a background
    std::string file_foreground_;///Could provide a 2d image to be used as a background

    /** Light source */
    std::vector<SLight> lights_;

    SGraphicsParsed() : SObject(std::string("SGraphicsParsed")){}
  };

}

#endif /* SGRAPHICSPARSED_HPP_ */

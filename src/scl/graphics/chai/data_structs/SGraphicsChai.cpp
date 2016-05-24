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
/* \file SGraphicsChai.cpp
 *
 *  Created on: Jul 18, 2015
 *
 *  Copyright (C) 2015
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#include "SGraphicsChai.hpp"

#include "chai3d.h"

#include <stdexcept>
#include <iostream>

namespace scl
{
  bool SGraphicsChai::init(const SGraphicsParsed* arg_gr_ds, const bool arg_reset_chai_world)
  {
    bool chai_ds_deleted_ = false;
    bool flag;
    try
    {
      if(S_NULL == arg_gr_ds)
      { throw(std::runtime_error("Passed NULL object for the parsed graphics specification.")); }

      // We'll reset the initialization flag right here..
      has_been_init_ = false;

#ifdef DEBUG
      if(S_NULL != chai_cam_ || S_NULL != chai_world_)
      { std::cout<<"\nSGraphicsChai::init() : Chai data structures are already initialized. Option to reset = "<<arg_reset_chai_world;  }
#endif

      // Decide whether to reset the chai world..
      if(arg_reset_chai_world)
      {
        if(S_NULL!= chai_world_) { delete chai_world_; chai_world_ = S_NULL; }
        if(S_NULL!= chai_cam_)   { delete chai_cam_; chai_cam_ = S_NULL; }
      }

      // Reallocate memory if we have the world set to a null pointers by the time we get here.
      if(S_NULL== chai_world_)
      {
        //Create a new world : Don't worry about deallocating it.
        chai_world_ = new chai3d::cWorld();
        if(S_NULL == chai_world_) { throw(std::runtime_error("Couldn't initialize chai world"));  }

        // NOTE: If the world is reset, the camera has to be reset..
        if(S_NULL!= chai_cam_)   { delete chai_cam_; chai_cam_ = S_NULL; }
      }
      if(S_NULL == chai_cam_)
      {
        //Create a camera
        chai_cam_ = new chai3d::cCamera(chai_world_);
        if(S_NULL == chai_cam_) { throw(std::runtime_error("Couldn't create a chai camera")); }
      }

      //Name it
      name_ = arg_gr_ds->name_;

      //Set up chai world and camera
      chai_world_->m_userName = "scl_id_chai_world";

      //Set the background color (R,G,B)
      chai_world_->setBackgroundColor(arg_gr_ds->background_color_[0],arg_gr_ds->background_color_[1], arg_gr_ds->background_color_[2]);

      //Insert the camera into the scenegraph world
      chai_world_->addChild(chai_cam_);

      //Set up the camera
      chai_cam_->m_userName = "scl_id_chai_cam";
      chai3d::cVector3d tmp1(arg_gr_ds->cam_pos_[0], arg_gr_ds->cam_pos_[1], arg_gr_ds->cam_pos_[2]);
      chai3d::cVector3d tmp2(arg_gr_ds->cam_lookat_[0], arg_gr_ds->cam_lookat_[1], arg_gr_ds->cam_lookat_[2]);
      chai3d::cVector3d tmp3(arg_gr_ds->cam_up_[0], arg_gr_ds->cam_up_[1], arg_gr_ds->cam_up_[2]);

      // Set up the camera
      chai_cam_->set(/** position (eye) */tmp1, /** lookat position (target)*/ tmp2,
          /** "up" direction */ tmp3);

#ifdef DEBUG
      std::cout<<"\nCam Pos    :"<<arg_gr_ds->cam_pos_[0]<<" "<<arg_gr_ds->cam_pos_[1]<<" "<<arg_gr_ds->cam_pos_[2];
      std::cout<<"\nCam Lookat :"<<arg_gr_ds->cam_lookat_[0]<<" "<<arg_gr_ds->cam_lookat_[1]<<" "<< arg_gr_ds->cam_lookat_[2];
      std::cout<<"\nCam Up     :"<<arg_gr_ds->cam_up_[0]<<" "<<arg_gr_ds->cam_up_[1]<<" "<< arg_gr_ds->cam_up_[2];
      std::cout<<"\nBack color : "<<arg_gr_ds->background_color_[0]<<", "
          <<arg_gr_ds->background_color_[1]<<", "<<arg_gr_ds->background_color_[2]<<std::endl;
#endif

      //Set the near and far clipping planes of the camera
      //Anything in front/behind these clipping planes will not be rendered
      chai_cam_->setClippingPlanes(arg_gr_ds->cam_clipping_dist_[0], arg_gr_ds->cam_clipping_dist_[1]);

      //Attach background/foreground images
      if(arg_gr_ds->file_background_ != "")
      {
        chai3d::cBackground *img = new chai3d::cBackground();
        flag = img->loadFromFile(arg_gr_ds->file_background_);
        if(false == flag)
        { std::cout<<"\nSGraphicsChai::init() : ERROR : Could not load background image. Proceeding...";  }
        else{
          // Attach the background to the camera's back layer. Chai's default
          // is to support "widgets" like this on layers (see chai code for more).
          chai_cam_->m_backLayer->addChild(img);
        }
      }
      //Attach background/foreground images
      if(arg_gr_ds->file_foreground_ != "")
      {
        chai3d::cBackground *img = new chai3d::cBackground();
        flag = img->loadFromFile(arg_gr_ds->file_foreground_);
        if(false == flag)
        { std::cout<<"\nSGraphicsChai::init() : ERROR : Could not load foreground image. Proceeding...";  }
        else{
          // Attach the foreground to the camera's back layer. Chai's default
          // is to support "widgets" like this on layers (see chai code for more).
          chai_cam_->m_frontLayer->addChild(img);
        }
      }

      // create a light source and attach it to the camera
      std::vector<SGraphicsParsed::SLight>::const_iterator it,ite;
      for(it = arg_gr_ds->lights_.begin(), ite = arg_gr_ds->lights_.end(); it!=ite; ++it)
      {
        chai3d::cDirectionalLight* light = new chai3d::cDirectionalLight(chai_world_);
        if(S_NULL==light)
        { throw(std::runtime_error("Couldn't add a light to the world")); }
        light->m_userName = std::string("scl_id_light_in_world");

        chai_cam_->addChild(light);                   // attach light to camera
        light->setEnabled(true);                   // enable light source

        chai3d::cVector3d pos( it->pos_[0], it->pos_[1], it->pos_[2]);
        chai3d::cVector3d lookat( it->lookat_[0], it->lookat_[1], it->lookat_[2]);

        light->setLocalPos(pos);  // position the light source
        light->setDir(lookat-pos);  // define the direction of the light beam

#ifdef DEBUG
        std::cout<<"\nLight Pos    :"<<pos(0)<<" "<<pos(1)<<" "<<pos(2);
        std::cout<<"\nLight Lookat :"<<lookat(0)<<" "<<lookat(1)<<" "<<lookat(2);
        std::cout<<"\nLight Dir    :"<<lookat(0)-pos(0)<<" "<<lookat(1)-pos(1)<<" "<<lookat(2)-pos(2)<<std::endl;
#endif
      }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nSGraphicsChai::init() : "<<ee.what();
      return false;
    }
    has_been_init_ = true;
    return true;
  }

}

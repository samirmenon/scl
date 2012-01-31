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
/* \file test_math.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_math.hpp"

#include <sutil/CSystemClock.hpp>
#include <scl/DataTypes.hpp>

//The math libraries to be tested
#include <Eigen/Dense>
//From chai
#include "math/CMatrix3d.h"

#include <string>
#include <iostream>
#include <vector>
#include <stdio.h>

namespace scl_test
{
  /**
   * Tests the performance of the available matrix libs
   * chai, eigen
   */
  void test_matrix_libs(int id)
  {
    scl::sUInt r_id=0;
    try
    {
      //0. Create vars
      long long i,jjj, jjj_max, imax;
      scl::sClock t1,t2;
      Eigen::Matrix3d e_m1,e_m2,e_m3; int eig_sz=3;
      cMatrix3d c_m1,c_m2,c_m3;

      //1. Init matrix and randomize
      //1.a) Eigen
      e_m1 = Eigen::MatrixXd::Random(eig_sz,eig_sz);
      e_m2 = Eigen::MatrixXd::Random(eig_sz,eig_sz);
      e_m3 = Eigen::MatrixXd::Random(eig_sz,eig_sz);

      std::cout<<"\nTest Result ("<<r_id++<<")  : Set up Eigen matrices :\n"
          <<e_m1<<"\n   to be mutiplied by:\n"<<e_m2<<std::endl;

      //1.b) Chai
      //Set up chai matrices to match the eigen matrices
      for(int i1=0;i1<3;i1++)
        for(int j1=0;j1<3;j1++)
        {
          c_m1.m[i1][j1] = e_m1(i1,j1);
          c_m2.m[i1][j1] = e_m2(i1,j1);
        }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Set up Chai matrices :";
      for(int i1=0;i1<3;i1++)
      {
        std::cout<<"\n";
        for(int j1=0;j1<3;j1++)
        {  std::cout<<c_m1.m[i1][j1]<<" ";  }
      }
      std::cout<<"\n   to be mutiplied by:";
      for(int i1=0;i1<3;i1++)
      {
        std::cout<<"\n";
        for(int j1=0;j1<3;j1++)
        {  std::cout<<c_m2.m[i1][j1]<<" ";  }
      }
      jjj_max = 1000;
      imax = 1000;

      std::cout<<"\n";

      //**************************
      //2.a. Test eigen
      //1,000,000 multiplies test
      t1 = sutil::CSystemClock::getSysTime();
      for(jjj=0;jjj<jjj_max;++jjj)
        for(i=0; i<imax; ++i)
        {
          e_m3 = e_m2*e_m1;
          e_m2.diagonal()*= 0.99999999;
        }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Eigen "<<jjj*i<<" matrix3d multiplications."
          <<"\n\tTime taken : "<<t2-t1<<"sec"
          <<"\n\tResulting matrix : ";
      std::cout<<"\n"<<e_m3<<std::endl;

      //**************************
      //2.b. Test chaimatrix
      //1,000,000 multiplies test
      t1 = sutil::CSystemClock::getSysTime();
      for(jjj=0;jjj<jjj_max;++jjj)
        for(i=0; i<imax; ++i)
        {
          c_m3 = c_m2*c_m1;
          c_m2.m[0][0] *= 0.99999999;
          c_m2.m[1][1] *= 0.99999999;
          c_m2.m[2][2] *= 0.99999999;
        }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Chai "<<jjj*i<<" matrix3d multiplications."
          <<"\n\tTime taken : "<<t2-t1<<"sec"
          <<"\n\tResulting matrix :";
      for(int i1=0;i1<3;i1++)
      {
        std::cout<<std::endl;
        for(int j1=0;j1<3;j1++)
        {  std::cout<<c_m3.m[i1][j1]<<" ";  }
      }

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<")  : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";
    }
  }

}

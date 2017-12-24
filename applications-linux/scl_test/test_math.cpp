/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

#include <string>
#include <iostream>
#include <vector>
#include <stdio.h>

namespace scl_test
{
  /**
   * Tests the performance of the available matrix libs
   * eigen
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

      //1. Init matrix and randomize
      //1.a) Eigen
      e_m1 = Eigen::MatrixXd::Random(eig_sz,eig_sz);
      e_m2 = Eigen::MatrixXd::Random(eig_sz,eig_sz);
      e_m3 = Eigen::MatrixXd::Random(eig_sz,eig_sz);

      std::cout<<"\nTest Result ("<<r_id++<<")  : Set up Eigen matrices :\n"
          <<e_m1<<"\n   to be mutiplied by:\n"<<e_m2<<std::endl;

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

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<")  : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";
    }
  }

}

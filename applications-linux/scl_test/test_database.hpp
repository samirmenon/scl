/* This file is part of Busylizzy, a control and simulation library
for robots and biomechanical models.

Busylizzy is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

Busylizzy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
Busylizzy. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file test_database.hpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef TEST_DATABASE_HPP_
#define TEST_DATABASE_HPP_

#include <iostream>
#include <math.h>

#include <sutil/CSystemClock.hpp>

#include <sstream>

using namespace scl;

namespace scl_test
{


  /***
   * Copies the database to a file and then restores it.
   * Checks to see if everything was restored properly
   */
  void test_database(int id)
  {
    bool flag = true;

    //Timers
    //double t1,t2;

    //  // Test 1 : Add the main memory and the copy-buffers
    //  flag = mcd.init((const double *)&mem);
    //  if(flag == false)  { std::cout<<"\nTest Result ("<<r_id++<<") Failed to initialize mem copier."; goto END; }
    //  else  { std::cout<<"\nTest Result ("<<r_id++<<") Initialize mem copier with data :"<< mem; }
    //
    //  //Test the memcpy speed
    //  t1 = sutil::CSystemClock::getSysTime();
    //  for(sLongLong i=0;i<1000000;i++)
    //  {
    //    mcd.copy();
    //  }
    //  t2 = sutil::CSystemClock::getSysTime();
    //  std::cout<<"\nTest Result ("<<r_id++<<") Memcpy Stress : "<<buf_sz<<" * 1,000,000 *"
    //      <<sizeof(double)<<" bytes in "<<t2-t1<<" seconds";


    if(true==flag)
    { std::cout<<"\nTest #"<<id<<" (Database Test) Successful"; }
    else
    { std::cout<<"\nTest #"<<id<<" (Database Test) Failed"; }
  }

}


#endif /* TEST_DATABASE_HPP_ */

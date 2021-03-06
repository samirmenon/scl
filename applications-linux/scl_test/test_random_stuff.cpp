/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file test_random_stuff.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#include "test_random_stuff.hpp"

#include <sutil/CSystemClock.hpp>

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <stdexcept>

//#define SCL_RANDOM_SLIM_TEST 1

namespace scl_random_tests
{
  class CSuper
  {
  protected:
    std::string mystr;
    double mydbl;
  public:
    virtual ~CSuper(){}

    virtual bool uselessVirtFunc(const std::string& arg)
    {
#ifndef SCL_RANDOM_SLIM_TEST
      mystr = arg;
#endif
      return false;
    }

    bool uselessNonVirtFunc(const std::string& arg)
    {
#ifndef SCL_RANDOM_SLIM_TEST
      mystr = arg;
#endif
      return false;
    }

    virtual bool uselessNonInlineVirtFunc(const std::string& arg);

    bool uselessNonInlineNonVirtFunc(const std::string& arg);

    void pointer0Redirect(const double arg)
    { mydbl = arg; }

    void pointer1Redirect(const double* arg)
    { mydbl = *arg; }

    void pointer2Redirect(const double** arg)
    { mydbl = **arg; }
  };

  bool CSuper::uselessNonInlineVirtFunc(const std::string& arg)
  {
#ifndef SCL_RANDOM_SLIM_TEST
    mystr = arg;
#endif
    return false;
  }

  bool CSuper::uselessNonInlineNonVirtFunc(const std::string& arg)
  {
#ifndef SCL_RANDOM_SLIM_TEST
    mystr = arg;
#endif
    return false;
  }

  class CSub : public CSuper
  {
  public:
    virtual ~CSub(){}

    virtual bool uselessVirtFunc(const std::string& arg)
    {
#ifndef SCL_RANDOM_SLIM_TEST
      mystr = arg;
#endif
      return true;
    }

    bool uselessNonVirtSubFunc(const std::string& arg)
    {
#ifndef SCL_RANDOM_SLIM_TEST
      mystr = arg;
#endif
      return false;
    }

    virtual bool uselessNonInlineVirtFunc(const std::string& arg);

    bool uselessNonInlineNonVirtFunc(const std::string& arg);
  };

  bool CSub::uselessNonInlineVirtFunc(const std::string& arg)
  {
#ifndef SCL_RANDOM_SLIM_TEST
    mystr = arg;
#endif
    return true;
  }

  bool CSub::uselessNonInlineNonVirtFunc(const std::string& arg)
  {
#ifndef SCL_RANDOM_SLIM_TEST
    mystr = arg;
#endif
    return true;
  }
}

namespace scl_test
{
  /**
   * Tests the robot parser with the sample Scl format
   *
   * Some interesting facts:
   *  1. function call = ~3e-9 sec (ie. ~3 clock cycles for a GHz proc)
   *  2. Virtual and non-virtual functions take pretty much the same time
   *     (In fact, virtual functions are sometimes faster).
   *  3. Pointer redirects take almost 0 time.
   */
  void test_random_stuff(int id)
  {
    int r_id=0;
    double t1,t2;
#ifndef SCL_RANDOM_SLIM_TEST
    const long long test_ctr = 100000;
#else
    const long long test_ctr = 1000000000;
#endif
    try
    {
      /* initialize random seed: */
      srand ( time(NULL) );

      scl_random_tests::CSuper o1;
      scl_random_tests::CSub o2;

      std::cout<<"\nRunning each test "<<test_ctr<<" times.";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o1.uselessVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Super inline virtual function. Time taken : "<<t2-t1<<"sec";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o2.uselessVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Sub   inline virtual function. Time taken : "<<t2-t1<<"sec";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o1.uselessNonVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Super inline non-virtual function. Time taken : "<<t2-t1<<"sec";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o2.uselessNonVirtSubFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Sub   inline non-virtual function. Time taken : "<<t2-t1<<"sec";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o1.uselessNonInlineVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Super non-inline virtual function. Time taken : "<<t2-t1<<"sec";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o2.uselessNonInlineVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Sub   non-inline virtual function. Time taken : "<<t2-t1<<"sec";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o1.uselessNonInlineNonVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Super non-inline non-virtual function. Time taken : "<<t2-t1<<"sec";

      //Test virtual function performance
      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o2.uselessNonInlineNonVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Sub   non-inline non-virtual function. Time taken : "<<t2-t1<<"sec";

      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o1.uselessNonVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Super throw away returned type (bool). Time taken : "<<t2-t1<<"sec";

      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
#ifndef SCL_RANDOM_SLIM_TEST
        std::stringstream ss;
        ss<<rand();
#endif
        std::string str;
#ifndef SCL_RANDOM_SLIM_TEST
        ss>>str;
#endif
        o1.uselessNonVirtFunc(str);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : Super keep returned type (bool).       Time taken : "<<t2-t1<<"sec";

      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
        double arg;
#ifndef SCL_RANDOM_SLIM_TEST
        arg = static_cast<double>(rand()/static_cast<double>(RAND_MAX));
#endif
        o1.pointer0Redirect(arg);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : 0 Pointer Redirects Time taken : "<<t2-t1<<"sec";

      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
        double arg;
#ifndef SCL_RANDOM_SLIM_TEST
        arg = static_cast<double>(rand()/static_cast<double>(RAND_MAX));
#endif
        o1.pointer1Redirect(&arg);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : 1 Pointer Redirects Time taken : "<<t2-t1<<"sec";

      t1 = sutil::CSystemClock::getSysTime();
      for(long long i=0;i<test_ctr;++i)
      {
        double arg; const double *arg2;
#ifndef SCL_RANDOM_SLIM_TEST
        arg = static_cast<double>(rand()/static_cast<double>(RAND_MAX));
#endif
        arg2 = & arg;
        o1.pointer2Redirect(&arg2);
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<")  : 2 Pointer Redirects Time taken : "<<t2-t1<<"sec";

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";
    }
  }
}

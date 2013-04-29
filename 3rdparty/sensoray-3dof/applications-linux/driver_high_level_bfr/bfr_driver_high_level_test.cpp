///////////////////////////////////////////////////////////////
// Module    : bfr_demo.c
// Function  : Autodetect and exercise 2608 and 2620 i/o modules.
// Target OS : Linux
///////////////////////////////////////////////////////////////

//For compiling the demo in cpp instead of c.
#ifdef __cplusplus
extern "C" {
#endif

  #include "app2600.h"    // Linux api to 2600 middleware

#ifdef __cplusplus
}
#endif

#include <sensoray/CBfrDriver.hpp>

#include <sutil/CSystemClock.hpp>

#include <string>
#include <stdio.h>
#include <iostream>
#include <math.h>

# define MAX_AMP_OUTPUT_CURRENT 2.0

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int main()
{
  bool flag=true;
  int   ctrl_cycles;  // Total control cycles
  double  t_start, t_mid, t_tot;    // Benchmark start time.

  flag = sutil::CSystemClock::start();
  if(false == flag)
  {
    std::cout<<"\nCould not start clock.\n";
    return 1;
  }

  bfr::CBfrDriver bfrio;   // Create a driver object

  flag = bfrio.init();                     // Initialize the driver
  if(false == flag)  { return 1; }

  flag = true;
  // TEST 2 : Test encoders
  double q0, q1, q2;
  double fq0, fq1, fq2;
  double fx, fy, fz;
  while(1)
  {
    std::cout<<"\nGC Forces > "<<std::flush;
    // The user inputs operational space forces
    //std::cin>>fx>>fy>>fz;
    // The user inputs gc forces
    std::cin>>fq0>>fq1>>fq2;

    //Print the force to be applied.
    std::cout<<"\nSending driver inputs : "<<fq0<<", "<<fq1<<", "<<fq2<<std::flush;

    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<500; i++)
    { flag = flag && bfrio.readGcAnglesAndCommandGcTorques(q0, q1, q2, fq0, fq1, fq2);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read & set force failed"; break;	}
    else
    { std::cout<<"\nEncoders read & force applied. Time taken = "<<t_tot<<"\n\n"<<std::flush; }
  }

  // Exit
  bfrio.shutdown();                       // Shut down the driver

  return 0;
}

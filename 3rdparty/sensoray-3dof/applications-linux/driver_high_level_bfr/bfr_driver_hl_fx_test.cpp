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

//To catch ctrl+c
#include <signal.h>
#include <stdlib.h>

static bool flag_servo_running = true;

void exit_handler(int s)
{ flag_servo_running = false; }

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int main()
{
  //Set up the ctrl+c handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  bool flag=true;
  double  t_start, t_mid, t_tot;    // Benchmark start time.

  flag = sutil::CSystemClock::start();
  if(false == flag)
  { std::cout<<"\nCould not start clock.\n"; return 1;  }

  bfr::CBfrDriver bfrio;   // Create a driver object

  flag = bfrio.init();                     // Initialize the driver
  if(false == flag)  { return 1; }

  flag = true;
  // TEST 2 : Test encoders
  double q0, q1, q2;
  double x, y, z;
  double fx, fy, fz;

  //Set forces to zero at the start. (Just in case).
  flag = flag && bfrio.readGCAnglesAndCommandGCForces(q0, q1, q2, 0, 0, 0);
  while(flag_servo_running)
  {
    std::cout<<"\nEE Forces > "<<std::flush;
    // The user inputs ee forces
    std::cin>>fx>>fy>>fz;

    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<500; i++)
    { flag = flag && bfrio.readEEPositionAndCommandEEForce(x, y, z, fx, fy, fz); }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    std::cout<<"\nServo rate (ticks/s) = "<<500.0/t_tot<<std::endl;

    // Set force back to zero
    flag = flag && bfrio.readGCAnglesAndCommandGCForces(q0, q1, q2, 0, 0, 0);

    if(false == flag)  { std::cout<<"\nError : Encoder read read & set force failed"; break;	}
  }

  t_tot = sutil::CSystemClock::getSysTime() - t_start;

  std::cout<<"\nEnding Force_ee test. \nTime = "<<t_tot<<std::endl;

  // Exit
  bfrio.shutdown();                       // Shut down the driver

  return 0;
}

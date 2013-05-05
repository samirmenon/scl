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

#define BFR_FILE_LOGGING

int main()
{
  //Set up the ctrl+c handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  bool flag=true;
  int   ctrl_cycles=0;  // Total control cycles
  double  t_start, t_tot;    // Benchmark start time.

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
  double x, y, z;
  double xi, yi, zi;
  bfrio.getEEZeroPosition(xi,yi,zi);

#ifdef BFR_FILE_LOGGING
  FILE *fp;
  fp = fopen("BFRPositionLog.txt","w");
  if(NULL == fp)
  {std::cout<<"\nCould not open log file\n"; bfrio.shutdown();  return 1;  }
#endif

  //Set forces to zero at the start. (Just in case).
  bfrio.readGCAnglesAndCommandGCForces(q0, q1, q2, 0, 0, 0);
  t_start = sutil::CSystemClock::getSysTime();
  while(flag_servo_running)
  {
    //Read the ee position + cause a servo tick
    flag = flag && bfrio.readEEPosition(x, y, z);
    if(false == flag)  { std::cout<<"\nError : Encoder read read & set force failed"; break;  }

    //Get the joint angles (no servo tick)
    bfrio.getGCPosition(q0, q1, q2);

    t_tot = sutil::CSystemClock::getSysTime() - t_start;

#ifdef BFR_FILE_LOGGING
    fprintf(fp,"\n%lf %lf %lf %lf %lf %lf %lf", t_tot, q0, q1, q2, x-xi, y-yi, z-zi);
#endif

    ctrl_cycles++;
    if(ctrl_cycles%50 == 0)
    {
      std::cout<<"\nAngles = "<<q0<<", "<<q1<<", "<<q2;
      std::cout<<"\nEE Pos = "<<x-xi<<", "<<y-yi<<", "<<z-zi;
      ctrl_cycles = 0;
    }
  }
  t_tot = sutil::CSystemClock::getSysTime() - t_start;

#ifdef BFR_FILE_LOGGING
  fclose(fp);
#endif

  std::cout<<"\nEnding encoder test. \nTime = "<<t_tot
      <<"\nServo rate (ticks/s) = "<<static_cast<double>(bfrio.getServoTicks())/t_tot<<std::endl;

  // Exit
  bfrio.shutdown();                       // Shut down the driver

  return 0;
}

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

#include <sensoray/CSensoray3DofIODriver.hpp>

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

  sensoray::CSensoray3DofIODriver sensorayio;   // Create a driver object

  flag = sensorayio.init();                     // Initialize the driver
  if(false == flag)  { return 1; }

  flag = true;
  // Run raw inputs
  long c0, c1,c2;
  double f0, f1, f2;
  while(1)
  {
    std::cout<<"\nForces > "<<std::flush;
    // The user provides raw inputs to the driver
    std::cin>>f0>>f1>>f2;

    //Double
    const double f0mult = 0.45, f1mult = 0.45, f2mult = 0.45;
    double driver0_des = f0mult*f0;
    double driver1_des = f1mult*f1;
    double driver2_des = f2mult*f2;

    // Motor polarity. ==> inputs = Nm ==> final = unit-input
    double f0 = -1.0*driver0_des;
    double f1 = -1.0*driver1_des;
    double f2 =  1.0*driver2_des;

    if(fabs(f0/f0mult)>MAX_AMP_OUTPUT_CURRENT)
    { std::cout<<"\n Max current limit breached on motor0 : "<< f0/f0mult; break;}
    if(fabs(f1/f1mult)>MAX_AMP_OUTPUT_CURRENT)
    { std::cout<<"\n Max current limit breached on motor1 : "<< f1/f1mult; break;}
    if(fabs(f2/f2mult)>MAX_AMP_OUTPUT_CURRENT)
    { std::cout<<"\n Max current limit breached on motor2 : "<< f2/f2mult; break;}

    flag = true;

    //Print the force to be applied.
    std::cout<<"\nSending driver inputs : "<<f0<<", "<<f1<<", "<<f2<<std::flush;

    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<3000; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, f0, f1, f2);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read & set force failed"; break;	}
    else
    { std::cout<<"\nEncoders read & force applied. Time taken = "<<t_tot<<"\n\n"<<std::flush; }
  }

  // Exit
  sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);
  sensorayio.shutdown();                       // Shut down the driver

  return 0;
}

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
  // TEST 2 : Test encoders
  long c0, c1,c2;
  double fx, fy, fz;
  while(1)
  {
    std::cout<<"\nForces > "<<std::flush;
    // The user inputs operational space forces
    std::cin>>fx>>fy>>fz;
    //Constant calibration
    const double fxmult = 1.0/0.915;
    fx = fx*fxmult;

    //Double
    double fgc0_des = -0.177*fx + 1.217*fz;
    double fgc1_des = 1.2*fy;
    double fgc2_des = -0.45*fx;

    std::cout<<"\nGc tau reqiured : "<<fgc0_des<<", "<<fgc1_des<<", "<<fgc2_des<<std::flush;

    const double gear0 = 30.0, gear1 = 20.0, gear2 = 20.0;
    double maxon0_des = fgc0_des / gear0;
    double maxon1_des = fgc1_des / gear1;
    double maxon2_des = fgc2_des / gear2;

    std::cout<<"\nMaxon tau reqiured : "<<maxon0_des<<", "<<maxon1_des<<", "<<maxon2_des<<std::flush;

    const double maxon_tau_per_amp = .0578;// = 57.8 / 1000
    double amp0_des = maxon0_des / maxon_tau_per_amp;
    double amp1_des = maxon1_des / maxon_tau_per_amp;
    double amp2_des = maxon2_des / maxon_tau_per_amp;

    std::cout<<"\nAmps reqiured : "<<amp0_des<<", "<<amp1_des<<", "<<amp2_des<<std::flush;

    //Amp output = i_to_a * i; //Amps per unit input
    const double i_to_a0 = 1.9846; //Amps / unit-input
    const double i_to_a1 = 1.9863;
    const double i_to_a2 = 2.0338;

    double driver0_des = amp0_des / i_to_a0;
    double driver1_des = amp1_des / i_to_a1;
    double driver2_des = amp2_des / i_to_a2;

    // Motor polarity. ==> inputs = Nm ==> final = unit-input
    double f0 = -1.0*driver0_des;
    double f1 = -1.0*driver1_des;
    double f2 =  1.0*driver2_des;

    if(fabs(amp0_des)>MAX_AMP_OUTPUT_CURRENT)
    { std::cout<<"\n Max current limit breached on motor0 : "<< amp0_des; break;}
    if(fabs(amp1_des)>MAX_AMP_OUTPUT_CURRENT)
    { std::cout<<"\n Max current limit breached on motor1 : "<< amp1_des; break;}
    if(fabs(amp2_des)>MAX_AMP_OUTPUT_CURRENT)
    { std::cout<<"\n Max current limit breached on motor2 : "<< amp2_des; break;}

    flag = true;

    //Print the force to be applied.
    std::cout<<"\nSending driver inputs : "<<f0<<", "<<f1<<", "<<f2<<std::flush;

    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<500; i++)
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

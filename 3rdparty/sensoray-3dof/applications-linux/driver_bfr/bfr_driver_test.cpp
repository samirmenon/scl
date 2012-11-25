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

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int main()
{
  bool flag;
  int   ctrl_cycles;  // Total control cycles
  time_t  t_start;    // Benchmark start time.
  double  t_tot;     // Benchmark elapsed time.

  flag = sutil::CSystemClock::start();
  if(false == flag)
  {
    std::cout<<"\nCould not start clock.\n";
    return 1;
  }

  sensoray::CSensoray3DofIODriver sensorayio;   // Create a driver object

  flag = sensorayio.init();                     // Initialize the driver
  if(false == flag)  { return 1; }

  // TEST 1 : Run a full control loop
  t_start = sutil::CSystemClock::getSysTime();
  ctrl_cycles = sensorayio.testDriver(100);    // Run control loop until terminated or error.
  t_tot = sutil::CSystemClock::getSysTime() - t_start;

  // Report benchmark results.
  printf( "\nControl loop cycles:    %d", ctrl_cycles );
  printf( "\nElapsed time (seconds): %lf", t_tot );
  printf( "\nAverage I/O cycle time (msec):  %.2f \n\n", t_tot / static_cast<double>(ctrl_cycles) * 1000.0 );

  // TEST 2 : Test encoders
  long c0, c1,c2;
  t_start = sutil::CSystemClock::getSysTime();
  for(int i=0;i<300; i++)
  { ctrl_cycles = sensorayio.readEncoders(c0, c1, c2);  }
  t_tot = sutil::CSystemClock::getSysTime() - t_start;
  std::cout<<"\nEncoders read 300 times. Time taken = "<<t_tot<<"\n\n"<<std::flush;

  for(int i=0;i<300; i++)
  { ctrl_cycles = sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 1.0, 0.0, 0.0);  }
  t_tot = sutil::CSystemClock::getSysTime() - t_start;
  std::cout<<"\nEncoders read 300 times & +x force. Time taken = "<<t_tot<<"\n\n"<<std::flush;

  for(int i=0;i<300; i++)
  { ctrl_cycles = sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 1.0, 0.0);  }
  t_tot = sutil::CSystemClock::getSysTime() - t_start;
  std::cout<<"\nEncoders read 300 times & +y force. Time taken = "<<t_tot<<"\n\n"<<std::flush;

  for(int i=0;i<300; i++)
  { ctrl_cycles = sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 1.0);  }
  t_tot = sutil::CSystemClock::getSysTime() - t_start;
  std::cout<<"\nEncoders read 300 times & +z force. Time taken = "<<t_tot<<"\n\n"<<std::flush;

  // Exit
  sensorayio.shutdown();                       // Shut down the driver

	return 0;
}

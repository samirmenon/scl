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

#include <string>
#include <stdio.h>

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int main()
{
  bool flag;
  int   ctrl_cycles;  // Total control cycles
  time_t  t_start;    // Benchmark start time.
  double  t_tot;     // Benchmark elapsed time.

  sensoray::CSensoray3DofIODriver sensorayio;   // Create a driver object

  flag = sensorayio.init();                     // Initialize the driver
  if(false == flag)  { return 1; }

  t_start = time( NULL );
  ctrl_cycles = sensorayio.ioControlLoop();    // Run control loop until terminated or error.
  t_tot = difftime( time( NULL ), t_start );

  sensorayio.shutdown();                       // Shut down the driver

  // Report benchmark results.
  printf( "\nControl loop cycles:    %d", ctrl_cycles );
  printf( "\nElapsed time (seconds): %lf", t_tot );
  printf( "\nAverage I/O cycle time (msec):  %.2f \n\n", t_tot / static_cast<double>(ctrl_cycles) * 1000.0 );

	return 0;
}

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
  sensoray::CSensoray3DofIODriver sensorayio;

  flag = sensorayio.init();
  if(false == flag)
  { return 1; }

  // Execute the i/o control loop until it terminates.
  sensorayio.ioControlMain();

  // Close the api library.
  S26_DriverClose();

	return 0;
}

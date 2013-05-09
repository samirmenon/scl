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
#include "SBFRParams.hpp"
#include <sutil/CSystemClock.hpp>

#include <string>
#include <stdio.h>
#include <iostream>

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

  // TEST 1 : Run a full control loop
  t_start = sutil::CSystemClock::getSysTime();

  flag = true;
  // TEST 2 : Test encoders
  long c0, c1,c2;
  if(sensorayio.modeEncoderOnly())
  {  printf( "\nError: Motors not connected");  }
  else
  {
	flag = true;
	double g0, g1, g2;
	bfr::bfrGravity(0,0,0,g0,g1,g2);
	g0 = g0*4;
    t_mid = sutil::CSystemClock::getSysTime();
    char ch='y';
    while(ch=='y')
    {
        printf( "\nGravity: %lf %lf %lf", g0, g1, g2);
    	for(int i=0;i<3000; i++)
    	{
    		if(g0>1){g0 = 1; }
    		if(g0<-1){g0 = -1; }
    		if(g1>1){g1 = 1; }
    		if(g1<-1){g1 = -1; }
    		if(g2>1){g2 = 1; }
    		if(g2<-1){g2 = -1; }
    		flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, g0, g1, g2);
    	}
    	std::cout<<"\nAmp force? +/-/q: "<<std::flush;
    	std::cin>>ch;
    	if(ch == '+')
    	{	g0 = g0 + 0.05;  }
    	if(ch == '-')
    	{	g0 = g0 - 0.05;  }
    	if(ch != 'q')
    	{	ch = 'y';  }
    }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & grav force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & grav force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }
  }
  // Exit
  sensorayio.shutdown();                       // Shut down the driver

  return 0;
}

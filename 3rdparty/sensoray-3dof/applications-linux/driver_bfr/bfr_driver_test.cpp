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
  flag = flag && sensorayio.testDriver(300,-0.5);    // Run control loop until terminated or error.
  t_tot = sutil::CSystemClock::getSysTime() - t_start;

  if(false == flag)  { std::cout<<"\nError : Driver test failed";	}

  // Report benchmark results.
  printf( "\nControl loop cycles:    %d", ctrl_cycles );
  printf( "\nElapsed time (seconds): %lf", t_tot );
  printf( "\nAverage I/O cycle time (msec):  %.2f \n\n", t_tot / static_cast<double>(ctrl_cycles) * 1000.0 );

  flag = true;
  // TEST 2 : Test encoders
  long c0, c1,c2;
  if(sensorayio.modeEncoderOnly())
  {
	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncoders(c0, c1, c2);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read failed";	}
    else
    { std::cout<<"\nEncoders read 300 times. Time taken = "<<t_tot<<"\n\n"<<std::flush; }
  }
  else
  {
	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & 0 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & 0 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 1, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & Motor [0] force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & Motor [0] force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & 0 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & 0 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, -1, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & Motor [0]*-1 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & Motor [0]*-1 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & 0 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & 0 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

    flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 1, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & Motor [1] force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & Motor [1] force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & 0 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & 0 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

    flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, -1, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & Motor [1]*-1 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & Motor [1]*-1 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & 0 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & 0 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

    flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { ctrl_cycles = sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 1);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & Motor [2] force failed";	}
    else
    {  std::cout<<"\nEncoders read 300 times & Motor [2] force. Time taken = "<<t_tot<<"\n\n"<<std::flush;  }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & 0 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & 0 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

    flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { ctrl_cycles = sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, -1);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & Motor [2]*-1 force failed";	}
    else
    {  std::cout<<"\nEncoders read 300 times & Motor [2]*-1 force. Time taken = "<<t_tot<<"\n\n"<<std::flush;  }

	flag = true;
    t_mid = sutil::CSystemClock::getSysTime();
    for(int i=0;i<300; i++)
    { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);  }
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)  { std::cout<<"\nError : Encoder read read 300 times & 0 force failed";	}
    else
    { std::cout<<"\nEncoders read 300 times & 0 force. Time taken = "<<t_tot<<"\n\n"<<std::flush; }
  }
  // Exit
  sensorayio.shutdown();                       // Shut down the driver

  return 0;
}

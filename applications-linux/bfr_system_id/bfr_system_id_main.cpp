///////////////////////////////////////////////////////////////
// Module    : bfr_system_id_main.cpp
// Function  : Run system identification on the bfr device
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
#include <scl/util/FileFunctions.hpp>

#include <string>
#include <stdio.h>
#include <iostream>

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  bool flag=true;
  int   ctrl_cycles;  // Total control cycles
  double  t_start, t_mid, t_end, t_tot;    // Benchmark start time.

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
  if(sensorayio.modeEncoderOnly())
  { std::cout<<"\nError : Encoder-only mode : Require motors to run system identification.\n"<<std::flush;  }
  else
  {
    const long sysid_stim_rows = 121082, sysid_stim_cols = 2;
    Eigen::MatrixXd sys_id_stimulus;

    //Read in stimulus file
    flag = scl_util::readEigenMatFromFile(sys_id_stimulus, sysid_stim_rows, sysid_stim_cols, "./sysid_stimulus.txt");

    //Some error checks
    if(false == flag)
    {
      std::cout<<"\nError : Could not read `./sysid_stimulus.txt`."
          <<"\nCan't run system identification program.";
      return 0;
    }

    //Start time
    t_start = sutil::CSystemClock::getSysTime();

    //Run the loop over the motors to estimate the system's responses
    for(int i=0;i<3;i++)
    {
      std::cout<<"\nRunning identification for motor id = "<<i;
      std::cout<<"\nTime to be taken 0 - "<<sys_id_stimulus(sysid_stim_rows-1,0)<<" sec.";

      FILE* fp;
      char ss[50],ch;
      sprintf(ss,"SysIdLog%d.log",i)
      std::cout<<"\Will save data to log file: "<<ss<<". Continue?\n>>y/n : ";
      std::cin>>ch;
      if('y'!=ch) { break; }

      fp = fopen(ss,"w");
      if(NULL == fp)
      {
        std::cout<<"\nError : Could not open `./"<<ss<<"` log file."
            <<"\n Can't run system identification program.";
        break;
      }

      t_mid = sutil::CSystemClock::getSysTime();
      t_end = sutil::CSystemClock::getSysTime();

      //Loop over the stimulus time
      long idx = 0;
      while(t_end - t_mid < sys_id_stimulus(sysid_stim_rows-1 /** matrix size = n-rows -1 */,0))
      {
        // Either time runs out or the index exceeds the matrix size.
        while((t_end - t_mid > sys_id_stimulus(idx,0)) && (idx < sysid_stim_rows) )
        { idx++;  }
        if(0==i)
        { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, sys_id_stimulus(idx,1), 0.0, 0.0);  }
        else if(1==i)
        { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, sys_id_stimulus(idx,1), 0.0);  }
        else
        { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, sys_id_stimulus(idx,1));  }

        fprintf(fp, "\n%d %lf %ld %ld %ld %lf",i, t_end-t_mid, c0, c1, c2, sys_id_stimulus(idx,1) );

        t_end = sutil::CSystemClock::getSysTime();
      }
      fclose(fp);
      t_tot = sutil::CSystemClock::getSysTime() - t_mid;
      if(false == flag)
      { std::cout<<"\nError : System Identification : Encoder read & motor control failed during execution";	}
      else
      { std::cout<<"\nSystem Identification : Succeeded for joint"<<i<<". Time taken = "<<t_tot<<"\n\n"<<std::flush; }
    }
  }
  // Exit
  sensorayio.shutdown();                       // Shut down the driver

  return 0;
}

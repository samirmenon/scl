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
#include <scl/util/FileFunctions.hpp>
#include <Eigen/Core>

#include <string>
#include <stdio.h>
#include <iostream>
#include <time.h>

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
bool currentDateTime(std::string arg_date) {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[128];
  tstruct = *localtime(&now);

  // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  arg_date = buf;
  return true;
}

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int main()
{
  bool flag;
  double  t_start, t_curr;    // Benchmark start time.

  flag = sutil::CSystemClock::start();
  if(false == flag)  { std::cout<<"\nCould not start clock.\n"; return 1;   }

  sensoray::CSensoray3DofIODriver sensorayio;   // Create a driver object

  flag = sensorayio.init();                     // Initialize the driver
  if(false == flag)  { return 1; }

  // Read the command line params to decide what file range to run the experiment on.
  std::string filebase = argv[1];
  int start_id, end_id;
  std::stringstream ss;
  ss<<argv[2]; ss>>start_id;
  ss<<argv[3]; ss>>end_id;

  printf("\nRunning haptic perception trials: %s%d.txt to %s%d.txt",filebase,start_id,filebase,end_id);

  Eigen::MatrixXd response_mat;
  response_mat.Zero(end_id-start_id+1,2);

  Eigen::MatrixXd task_mat;

  char ss[128], ch;
  for(int file_idx=start_id, resp_id=0; file_idx<=end_id; ++file_idx, ++resp_id)
  {
    sprintf(ss,"%s%d.txt",filebase, file_idx);

    flag = scl_util::readEigenMatFromFile(task_mat, ss);
    if(false == flag)
    { std::cout<<"\nError. Could not read file: "<<ss<<std::flush; break; }

    flag = true;

    //Initialize the timer.
    t_start = sutil::CSystemClock::getSysTime();
    t_curr = sutil::CSystemClock::getSysTime();

    int idx=0;
    while(t_curr - t_start < task_mat(task_mat.rows(),1))
    {
      flag = sensorayio.readEncodersAndCommandMotors(c0, c1, c2, task_mat(idx,2), task_mat(idx,3), task_mat(idx,4));
      if(false == flag)
      { std::cout<<"\nError : readEncodersAndCommandMotors() failed at t = "<<sutil::CSystemClock::getSysTime(); }

      // Move to next force
      t_curr = sutil::CSystemClock::getSysTime();
      if(t_curr - t_start > task_mat(idx,1)) { idx++;  }

      // Done with the test
      if(idx > task_mat.rows()) { break;  }
    }

    // 1 = same. 0 = different.
    double were_forces_similar;
    std::cin>>were_forces_similar;
    response_mat(resp_id,1) = file_idx;
    response_mat(resp_id,2) = were_forces_similar;
  }

  std::string date = currentDateTime();
  sprintf(ss,"%sResponses_Human.txt",date.c_str());
  flag = scl_util::writeEigenMatToFile(response_mat, ss);
  if(false == flag)
  { std::cout<<"\nError. Could not read file: "<<ss<<std::flush; break; }

  // Exit
  sensorayio.shutdown();                       // Shut down the driver

  return 0;
}

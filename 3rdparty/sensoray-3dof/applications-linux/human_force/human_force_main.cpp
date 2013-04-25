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

#include <iomanip>
#include <string>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <time.h>

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
bool currentDateTime(std::string& arg_date) {
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

int main(int argc, char** argv)
{
  if(argc < 4)
  {
    std::cout<<"\nhuman_force application tests haptic perception capabilities."
        <<"\nThe command line input is: ./<executable> <dir/file_base> <id_start> <id_end>\n";
    return 0;
  }

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
  std::stringstream sstr;
  sstr<<argv[2]<<" ";
  sstr<<argv[3];
  sstr>>start_id;
  sstr>>end_id;
  //Error check.
  if(end_id<start_id)
  { std::cout<<"\nEnd id < start id. End = "<<end_id<<". Start = "<<start_id<<"\n"; return 1;   }
  else
  { std::cout<<"\nRunning experiment. End = "<<end_id<<". Start = "<<start_id<<"\n"; }

  printf("\nRunning haptic perception trials: %s%d.txt to %s%d.txt",filebase.c_str(),start_id,filebase.c_str(),end_id);

  Eigen::MatrixXd response_mat;
  response_mat.setZero(end_id-start_id+1,2);
  std::cout<<"\nResponse matrix initialized. Size ["<<response_mat.rows()<<", "<<response_mat.cols()<<"]"<<std::flush;

  char ss[128];
  for(int file_idx=start_id, resp_id=0; file_idx<=end_id; ++file_idx, ++resp_id)
  {
    Eigen::MatrixXd task_mat;

    sprintf(ss,"%s%d.txt",filebase.c_str(), file_idx);

    flag = scl_util::readEigenMatFromFile(task_mat, ss);
    if(false == flag)
    { std::cout<<"\nError. Could not read file: "<<ss<<std::flush; break; }
#ifdef DEBUG
    else
    { std::cout<<"\nRunning trial for: \n"<<task_mat<<std::flush; }
#endif

    flag = true;

    //Initialize the timer.
    t_start = sutil::CSystemClock::getSysTime();
    t_curr = sutil::CSystemClock::getSysTime();

    int idx=0;
    long c0, c1, c2;
    std::cout<<"\nRunning trial ["<<resp_id<<" / "<<response_mat.rows()-1<<"]. Time = "<<task_mat(task_mat.rows()-1,0)<<std::endl<<std::flush;
    while(t_curr - t_start < task_mat(task_mat.rows()-1,0))
    {
      //Add the force calibration
      // Either time runs out or the index exceeds the matrix size.
      const double force_multiplier=0.45;
      double f0 = -1*force_multiplier*task_mat(idx,1);
      double f1 = -1*force_multiplier*task_mat(idx,2);
      double f2 = force_multiplier*task_mat(idx,3);
      flag = sensorayio.readEncodersAndCommandMotors(c0, c1, c2, f0, f1, f2);
      if(false == flag)
      { std::cout<<"\nError : readEncodersAndCommandMotors() failed at t = "<<sutil::CSystemClock::getSysTime(); break; }

      // Move to next force
      t_curr = sutil::CSystemClock::getSysTime();
      while(t_curr - t_start > task_mat(idx,0)) {
          idx++;
          // Done with the test
          if(idx >= task_mat.rows()) { break;  }
      }
#ifdef DEBUG
      printf("\nForce : [%ld: %5.2lf %5.3lf, %5.3lf, %5.3lf]", idx, t_curr-t_start, f0, f1, f2);
#endif
    }

    // 1 = same. 0 = different.
    double force_response_val;
    std::cout<<"\nResponse ["<<resp_id
        <<"]\n\t0 Felt nothing"
        <<"\n\t 1 First was larger"
        <<"\n\t 2 Second was larger"
        <<"\n\t 3 Both were equal"
        <<"\n\t-1 Couldn't feel 1"
        <<"\n\t-2 Couldn't feel 2"
        <<"\n\t 9 Break\n >>"<<std::flush;
    std::cin>>force_response_val;
    if(9 == static_cast<int>(force_response_val))
    { break; }
    response_mat(resp_id,0) = file_idx;
    response_mat(resp_id,1) = force_response_val;
  }

  std::string date;
  currentDateTime(date);
  sprintf(ss,"%sResponses_Human.txt",date.c_str());
  flag = scl_util::writeEigenMatToFile(response_mat, ss);
  if(false == flag)
  { std::cout<<"\nError. Could not write response matrix to file: "<<ss<<std::flush; }

  std::cout<<"\nResponses: \n >>"<<response_mat<<"\n";

  // Exit
  sensorayio.shutdown();                       // Shut down the driver

  return 0;
}

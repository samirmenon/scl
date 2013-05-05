///////////////////////////////////////////////////////////////
// Module    : bfr_force_id_main.cpp
// Function  : Run force identification on the bfr device
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

#include <sensoray/CBfrDriver.hpp>

#include <sutil/CSystemClock.hpp>
#include <scl/util/FileFunctions.hpp>

#include <string>
#include <stdio.h>
#include <iostream>

//To catch ctrl+c
#include <signal.h>
#include <stdlib.h>

static bool flag_servo_running = true;

void exit_handler(int s)
{ flag_servo_running = false; }



#include <netinet/in.h>
#include <netdb.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>

#define PORT "3490"  // the port users will be connecting to

#define BACKLOG 10     // how many pending connections queue will hold

#define MAXDATASIZE 100 // max number of bytes we can get at once 

// EXECUTABLE ////////////////////////////////////////////////////////////////////

int receive(char *IP)
{
    int sockfd, numbytes;
    char buf[MAXDATASIZE];
    struct addrinfo hints, *servinfo, *p;
    int rv;
    char s[INET6_ADDRSTRLEN];

    /*if (argc != 2) {
        fprintf(stderr,"usage: client hostname\n");
        exit(1);
    }*/

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    //printf("\nTrying to get connection to %s", IP); fflush(NULL);

    if ((rv = getaddrinfo(IP, PORT, &hints, &servinfo)) != 0) {
        //fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
				printf("\nError..."); fflush(NULL);
        return 1;
    }

    // loop through all the results and connect to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("client: socket");
            continue;
        }

        if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("client: connect");
            continue;
        }

        break;
    }

    if (p == NULL) {
        //fprintf(stderr, "client: failed to connect\n");
				//printf("\nNo connection..."); fflush(NULL);
				close(sockfd);
        return 1;
    }

    /*inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
            s, sizeof s);
    printf("client: connecting to %s\n", s);*/

    freeaddrinfo(servinfo); // all done with this structure

    /*if ((numbytes = recv(sockfd, buf, MAXDATASIZE-1, 0)) == -1) {
        perror("recv");
        exit(1);
    }*/

    buf[numbytes] = '\0';

    printf("client: received '%s'\n",buf);

    close(sockfd);

    return 0;
}

int main(int argc, char** argv)
{
  //Set up the ctrl+c handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  if(argc != 3) {
    printf("\nERROR: Usage is (%d) \"./bfr_force_id <Sync Server IP> <sysid filename>\"\n",argc);
    return 0;
  }

  bool flag=true;
  int   ctrl_cycles;  // Total control cycles
  double  t_start, t_mid, t_end, t_tot;    // Benchmark start time.

  flag = sutil::CSystemClock::start();
  if(false == flag)
  {
    std::cout<<"\nCould not start clock.\n";
    return 1;
  }

  //sensoray::CSensoray3DofIODriver bfrio;   // Create a driver object
  bfr::CBfrDriver bfrio;   // Create a driver object

  flag = bfrio.init();                     // Initialize the driver
  if(false == flag)  { return 1; }

  flag = true;
  // TEST 2 : Test encoders
  double q0, q1, q2;
  if(bfrio.modePositionOnly())
  { std::cout<<"\nError : Encoder-only mode : Require motors to run Force identification.\n"<<std::flush;  }
  else
  {
    Eigen::MatrixXd sys_id_stimulus;

    //Read in stimulus file
    printf("\nReading file"); fflush(NULL);
    flag = scl_util::readEigenMatFromFile(sys_id_stimulus, argv[2]);
    printf("\nRead file"); fflush(NULL);
    
    //Some error checks
    if(false == flag)
    {
      std::cout<<"\nError : Could not read `./sysid_stimulus.txt`."
          <<"\nCan't run Force identification program.";
      return 0;
    }

    std::cout<<"\nTime to be taken 0 - "<<sys_id_stimulus(sys_id_stimulus.rows()-1,0)<<" sec.";

    FILE* fp;
    char ss[50];
    std::cout<<"\nWill save data to log file: "<<"ForceIdLog.log"<<std::flush;

    fp = fopen("ForceIdLog.log","a");
    if(NULL == fp)
    {
    std::cout<<"\nError : Could not open `./"<<ss<<"` log file."
    <<"\n Can't run Force identification program.";
    goto DELETEMELATER; //break;
    }
    fprintf(fp,"\n***********************************\n***********************************\n");


    printf("\nAwaiting server..."); fflush(NULL);
    while(receive(argv[1])) {
    //printf("\nAwaiting server..."); fflush(NULL);
    }

    //Start time
    t_start = sutil::CSystemClock::getSysTime();

    t_mid = sutil::CSystemClock::getSysTime();
    t_end = sutil::CSystemClock::getSysTime();

    //Loop over the stimulus time
    long idx = 0;
    long sysid_stim_rows = sys_id_stimulus.rows();
    while(t_end - t_mid < sys_id_stimulus(sysid_stim_rows-1 /** matrix size = n-rows -1 */,0))
    {
      // Either time runs out or the index exceeds the matrix size.
      const double force_multiplier=1.0;//0.45;
      while((t_end - t_mid > sys_id_stimulus(idx,0)) && (idx < sysid_stim_rows) )
      { idx++;  }

      double f0 = force_multiplier*sys_id_stimulus(idx,1);
      double f1 = force_multiplier*sys_id_stimulus(idx,2);
      double f2 = force_multiplier*sys_id_stimulus(idx,3);
      flag = flag && bfrio.readGCAnglesAndCommandGCForces(q0, q1, q2, f0, f1, f2);

      fprintf(fp, "\n%d %lf %lf %lf %lf %lf %lf %lf",-1, t_end-t_start, q0, q1, q2, force_multiplier*sys_id_stimulus(idx,1), force_multiplier*sys_id_stimulus(idx,2), force_multiplier*sys_id_stimulus(idx,3));

      t_end = sutil::CSystemClock::getSysTime();

      if(false == flag_servo_running) break; //ctrl+c
    }
    fclose(fp);
    t_tot = sutil::CSystemClock::getSysTime() - t_mid;
    if(false == flag)
    { std::cout<<"\nError : Force Identification : Encoder read & motor control failed during execution";	}
    else
    { std::cout<<"\nForce Identification : Succeeded. Time taken = "<<t_tot<<"\n\n"<<std::flush; }

  }
DELETEMELATER:
  // Exit
  bfrio.shutdown();                       // Shut down the driver

  return 0;
}

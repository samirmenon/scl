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

  if(argc != 4) {
    printf("\nERROR: Usage is \"./bfr_system_id <Sync Server IP> <Motor #> <sysid filename>\"\n");
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

  printf("\nAwaiting server..."); fflush(NULL);
  while(receive(argv[1])) {
    //printf("\nAwaiting server..."); fflush(NULL);
  }

  //Start time
  t_start = sutil::CSystemClock::getSysTime();

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
    //const long sysid_stim_rows = 121082, sysid_stim_cols = 2; //Original 10Hz
    //const long sysid_stim_rows = 1496884, sysid_stim_cols = 2; // 200Hz
    //const long sysid_stim_rows = 547403, sysid_stim_cols = 2; // 65Hz (obsolete. now just read file)
    Eigen::MatrixXd sys_id_stimulus;

    //Read in stimulus file
	printf("\nReading file"); fflush(NULL);
    flag = scl_util::readEigenMatFromFile(sys_id_stimulus, argv[3]);

    //Some error checks
    if(false == flag)
    {
      std::cout<<"\nError : Could not read `./sysid_stimulus.txt`."
          <<"\nCan't run system identification program.";
      return 0;
    }

    //Run the loop over the motors to estimate the system's responses
	printf("\nGetting motor number"); fflush(NULL);
	int i;	
	switch(*argv[2])
	{
	  case '0':
		i = 0;
		break;
	  case '1':
		i = 1;
		break;
	  case '2':
		i = 2;
		break;
	  default:
		i = 0;
	}

    for(i=0;i<3;i++)
    {
      std::cout<<"\nRunning identification for motor id = "<<i;
      std::cout<<"\nTime to be taken 0 - "<<sys_id_stimulus(sys_id_stimulus.rows()-1,0)<<" sec.";

      FILE* fp;
      char ss[50],ch;
      sprintf(ss,"SysIdLog%d.log",i);
      std::cout<<"\Will save data to log file: "<<ss;

      fp = fopen(ss,"a");
      if(NULL == fp)
      {
        std::cout<<"\nError : Could not open `./"<<ss<<"` log file."
            <<"\n Can't run system identification program.";
        break;
      }
      fprintf(fp,"\n***********************************\n***********************************\n");

//      std::cout<<". Continue?\n>>y/n : ";
//      std::cin>>ch;
//      if('y'!=ch) { break; }

      t_mid = sutil::CSystemClock::getSysTime();
      t_end = sutil::CSystemClock::getSysTime();

      //Loop over the stimulus time
      long idx = 0;
      long sysid_stim_rows = sys_id_stimulus.rows();
      while(t_end - t_mid < sys_id_stimulus(sysid_stim_rows-1 /** matrix size = n-rows -1 */,0))
      {
        // Either time runs out or the index exceeds the matrix size.
        while((t_end - t_mid > sys_id_stimulus(idx,0)) && (idx < sysid_stim_rows) )
        { idx++;  }
        if(0==i)
        { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.33*sys_id_stimulus(idx,1), 0.0, 0.0);  }
        else if(1==i)
        { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.33*sys_id_stimulus(idx,1), 0.0);  }
        else
        { flag = flag && sensorayio.readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.33*sys_id_stimulus(idx,1));  }

        fprintf(fp, "\n%d %lf %ld %ld %ld %lf",i, t_end-t_start, c0, c1, c2, sys_id_stimulus(idx,1) );

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

/*
 * scl_nw_conn.cpp
 *
 *  Created on: Oct 17, 2011
 *      Author: Samir Menon
 */

#include <iostream>
#include <stdexcept>

#include <PracticalSocket.h>

#include <Eigen/Core>

#include <sutil/CSystemClock.hpp>

//Client:
//#define N_TRAN 10
//#define N_RECV 300

//Server:
#define N_TRAN 300
#define N_RECV 10


/** A sample network server app */
int main(int argc, char** argv)
{
  if(argc != 1)
  {
    std::cout<<"\nscl_nw_server : Sample network server\n";
    return 0;
  }
  else
  {
    try
    {
      std::cout<<"\nStart Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      //Set up a client socket:
      TCPServerSocket cli_serv("127.0.0.1",8081);

      double t[N_TRAN]; //Transmit 300 values
      double r[N_RECV]; //Recv 10 values

      //Wait to accept the other person's socket.
      TCPSocket* conn_sock = cli_serv.accept();
      std::cout<<"\nscl_nw_server : Client connected.";

      //Then send/recv the stuff: 10 values, 10 times
      for(unsigned int i=0;i<10;++i)
      {
        r[0] = N_RECV;
        conn_sock->recv(r,N_RECV*sizeof(double));
        std::cout<<"\nServ : Recv ["<<N_RECV<<" doubles] : ";
        for(unsigned int j=0;j<N_RECV;++j)
          std::cout<<r[j]<<", ";

        //First set the 10 values to the time stamps
        t[0] = N_TRAN;
        for(unsigned int j=1;j<N_TRAN;++j)
        { t[j] = sutil::CSystemClock::getSysTime(); }

        conn_sock->send(t,N_TRAN*sizeof(double));
        std::cout<<"\nServ : Sent ["<<N_TRAN<<" doubles] : ";
        for(unsigned int j=0;j<N_TRAN;++j)
          std::cout<<t[j]<<", ";
      }

      std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime()<<"\n";
      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      std::cout<<"\nSCL Failed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}

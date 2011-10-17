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

#define N_TRAN 10
#define N_RECV 300


/** A sample network communication app */
int main(int argc, char** argv)
{
  bool flag;
  if(argc != 1)
  {
    std::cout<<"\nscl_nw_conn : Sample network communication\n";
    return 0;
  }
  else
  {
    try
    {
      std::cout<<"\nStart Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      //Set up a client socket:
      TCPSocket cli_sock("172.0.0.1",8080);

      double t[N_TRAN]; //Transmit 10 values
      double r[N_RECV]; //Recv 300 values

      //Then send/recv the stuff: 10 values, 10 times
      for(unsigned int i=0;i<10;++i)
      {
        //First set the 10 values to the time stamps
        t[0] = N_TRAN;
        for(unsigned int j=1;j<N_TRAN;++j)
        { t[j] = sutil::CSystemClock::getSysTime(); }

        flag = cli_sock.send(t,N_TRAN*sizeof(double));
        if (true == flag)
        {
          std::cout<<"\nSent ["<<N_TRAN<<" doubles] : ";
          for(unsigned int j=0;j<N_TRAN;++j)
            std::cout<<t[j]<<", ";
        }
        else{ throw("Send failed"); }

        r[0] = N_RECV;
        flag = cli_sock.recv(r,N_RECV*sizeof(double));
        if (true == flag)
        {
          std::cout<<"\nRecv ["<<N_RECV<<" doubles] : ";
          for(unsigned int j=0;j<N_RECV;++j)
            std::cout<<r[j]<<", ";
        }
        else{ throw("Recv failed"); }
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

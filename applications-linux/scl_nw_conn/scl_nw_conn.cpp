/*
 * scl_nw_conn.cpp
 *
 *  Created on: Oct 17, 2011
 *      Author: Samir Menon
 */

#include <iostream>
#include <stdexcept>

#include <PracticalSocket.h>

#include <scl/CSystemClock.hpp>


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

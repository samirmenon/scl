/* This file is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

This file is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
this file. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file HelperFunctions.cpp
 *
 *  Created on: Dec 26, 2010
 *
 *  Copyright (C) 2010, Samir Menon <smenon@stanford.edu>
 */

#include <scl/util/HelperFunctions.hpp>

#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <ctype.h>
#ifndef WIN32
//For getting the current working directory.
#include <unistd.h>
#endif

namespace scl_util
{

  unsigned int countNumbersInString(const char* arg_str)
  {
    if(arg_str==NULL)
    { return 0; }

    unsigned int ctr = 0, sz=0;
    //Assume numbers are separated by whitespace.
    while(arg_str[ctr]!='\0')
    {
      // Iterate till a number appears.
      while(std::isspace(arg_str[ctr]))
      {
        ctr++;
        if(arg_str[ctr]=='\0')
        { return sz;  }
      }
      sz++;//Found a number (presumably)

      // Now iterate till the end of the number.
      while(false == std::isspace(arg_str[ctr]))
      {
        ctr++;
        if(arg_str[ctr]=='\0')
        { return sz;  }
      }
    }// End of while loop over string.
    return sz;
  }

  bool isStringInVector(const std::string& arg_str,
        const std::vector<std::string>& arg_vec)
  {
    try
    {
      if(arg_vec.size()<=0)
      { throw(std::runtime_error("Passed vector has no strings"));  }
      if(arg_str.length()<=0)
      { throw(std::runtime_error("Passed string has no characters"));  }

      std::vector<std::string>::const_iterator it, ite;
      for(it = arg_vec.begin(), ite = arg_vec.end();it!=ite; ++it)
      { if(arg_str == *it) { return true;  } }
    }
    catch(std::exception& e)
    { std::cout<<"\nisStringInVector() Error : "<<e.what(); }
    return false;
  }

  bool getCurrentDir(std::string& arg_cwd)
  {
    try
    {
#ifndef WIN32
      //Set current working directory.
      char path[2048];
      if(NULL == getcwd(path,2048))
      { throw(std::runtime_error("Could not read current working directory"));  }
      arg_cwd = path;
      arg_cwd = arg_cwd + "/";
#else
      //NOTE TODO : Implement a version for windows
      arg_cwd = "";
#endif
      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\ngetCurrentDir() Error : "<<e.what(); }
    return false;
  }
}

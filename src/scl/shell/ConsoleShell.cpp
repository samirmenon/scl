/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
 */
/* \file CConsoleShell.cpp
 *
 *  Created on: Aug 21, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "ConsoleShell.hpp"
#include <sutil/CRegisteredCallbacks.hpp>

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <string>
//String tokenizer uses std::algorithm
#include <algorithm>
#include <iterator>

#include <stdio.h>
#include <ncurses.h>

namespace scl
{
  namespace shell
  {
    void tokenizeCharArr(const char* arg_arr, std::vector<std::string> &ret_vec)
    {
      ret_vec.clear();
      //Split the string into string tokens
      std::istringstream iss(arg_arr);
      std::copy(std::istream_iterator<std::string>(iss),
          std::istream_iterator<std::string>(),
          std::back_inserter<std::vector<std::string> >(ret_vec));
    }

    void runConsoleShell(scl::SDatabase &arg_db)
    {
      bool flag, flag2;
      bool mode_char = false;

      using namespace std;

      std::vector<std::string> last_command;
      std::vector<std::string> tokens;

      char cmd_buf[1024];

      while(arg_db.running_)
      {
        while(arg_db.running_ && mode_char == false)
        {
          std::cout<<"\nscl>> ";

          // Get the input string(s)
          char* ret = fgets(cmd_buf,1024,stdin);
          if(ret!=cmd_buf)  { std::cout<<"\nError in shell: Could not read stdin";  }

          // Tokenize the string so we can parse it easily.
          tokenizeCharArr(cmd_buf, tokens);

          //Some error checks
          if(tokens.begin() == tokens.end())
          {//Pressing enter executes the last command
            if(0==last_command.size())
            { std::cout<<"Command not found"; continue; }
            else
            { tokens = last_command; }
          }

          last_command = tokens;//Set the last command

          if(std::string("exit") == *(tokens.begin()) )
          {
            arg_db.running_ = false;
            break;
          }
          else if(std::string("x") == *(tokens.begin()) )
          { mode_char = true; std::cout<<"  Switched to char mode and back"; continue; }
          else if(std::string("p") == *(tokens.begin()) )
          {//Toggle pause
            if(arg_db.pause_ctrl_dyn_)
            { std::cout<<"  Un-paused controller and dynamics engine"; }
            else { std::cout<<"  Paused controller and dynamics engine"; }

            arg_db.pause_ctrl_dyn_= !arg_db.pause_ctrl_dyn_;
            continue;
          }

          //Find and call the callback
          flag = sutil::callbacks::call<std::string, std::vector<std::string> >(
              *(tokens.begin()),tokens);
          if(false == flag) { std::cout<<"Command not found"; }
        }

        initscr(); //Start ncurses (the library for console io)
        raw();     //Disable line buffering (disable the enter key press)
        printw("  Char mode (press x to exit)\n\n>>");
        while(arg_db.running_ &&
            mode_char == true)
        {
          char input='1';
          input = getch();

          if('x' == tolower(input)){ mode_char = false; continue; }
          if('p' == tolower(input))
          {arg_db.pause_ctrl_dyn_= !arg_db.pause_ctrl_dyn_;
          continue;}

          flag2 = tolower(input) != input;
          flag = sutil::callbacks::call<char,bool,double>(tolower(input), flag2);
          if(false == flag)
          {
            flag = sutil::callbacks::call<char,bool,Eigen::VectorXd>(tolower(input), flag2);
            if(false == flag) { std::cout<<" NotFound "; }
          }
        }
        endwin(); //Stop ncurses
      }
    }

  } /* namespace shell */
} /* namespace scl */

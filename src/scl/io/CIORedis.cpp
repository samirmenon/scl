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
/*
 * CIORedis.cpp
 *
 *  Created on: Jul 10, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CIORedis.hpp"

#include <iostream>

namespace scl
{
  /** Connects to the Redis database using parameters specified in the
   * data structure. Also stores the connection details in the data
   * structure. */
  bool CIORedis::connect(SIORedis &arg_ds, bool arg_ping_server)
  {
    // Connect to the redis server
    arg_ds.context_= redisConnectWithTimeout(arg_ds.hostname_, arg_ds.port_, arg_ds.timeout_);
    if (arg_ds.context_ == NULL)
    {
      std::cout<<"\n\n CIORedis::connect() : ERROR : Could not allocate Redis context (could not connect to server)\n";
      return false;
    }

    if(arg_ds.context_->err)
    {
      std::cout<<"\n\n CIORedis::connect() : ERROR : Connection to Redis server resulted in an error\n";
      redisFree(arg_ds.context_);
      return false;
    }

    if(arg_ping_server)
    {
      bool server_replied=false;
      // PING server to make sure things are working..
      arg_ds.reply_ = (redisReply *)redisCommand(arg_ds.context_,"PING");
      if(arg_ds.reply_->len > 0){ server_replied = true; }
      freeReplyObject((void*)arg_ds.reply_);

      if(server_replied)
      { std::cout<<"\n\n CIORedis::connect() : Redis server is live. Reply to PING is, "<<arg_ds.reply_->str<<"\n"; }
      else
      {
        std::cout<<"\n\n CIORedis::connect() : ERROR : Redis server did not reply to ping\n";
        return false;
      }
    }

    // Connection successful.
    return true;
  }

  /** Sends a message.
  bool CIORedis::runCommand(SIORedis &arg_ds, char* arg_msg)
  {
    bool flag=false;
    arg_ds.reply_ = (redisReply *)redisCommand(arg_ds.context_, "%s", arg_msg);
    if(arg_ds.reply_->len > 0){ flag = true;  }//Succeeded
    freeReplyObject((void*)arg_ds.reply_);
    return flag;
  }*/

  /** Sets an Eigen vector as a string key. */
  bool CIORedis::set(SIORedis &arg_ds, const char* arg_key, const Eigen::VectorXd &arg_vec)
  {
    bool flag=false;
    // NOTE TODO : Probably faster to use sprintf.
    std::stringstream ss;
    for(int i=0;i<arg_vec.rows();++i){ ss<<arg_vec(i)<<" "; }
    sprintf(arg_ds.str_, "%s", ss.str().c_str());

    // Set the key
    arg_ds.reply_ = (redisReply *)redisCommand(arg_ds.context_, "SET %s %s",arg_key,arg_ds.str_);
    if(arg_ds.reply_->len > 0){ flag = true;  }//Succeeded
    freeReplyObject((void*)arg_ds.reply_);     //Clean up

    return flag;
  }

  /** Sets an Eigen vector as a string key. */
  bool CIORedis::set(SIORedis &arg_ds, const char* arg_key, const int arg_int)
  {
    bool flag=false;
    // NOTE TODO : Probably faster to use sprintf.
    std::stringstream ss; ss<<arg_int;
    sprintf(arg_ds.str_, "%s", ss.str().c_str());

    // Set the key
    arg_ds.reply_ = (redisReply *)redisCommand(arg_ds.context_, "SET %s %s",arg_key,arg_ds.str_);
    if(arg_ds.reply_->len > 0){ flag = true;  }//Succeeded
    freeReplyObject((void*)arg_ds.reply_);     //Clean up

    return flag;
  }

  /** Sets an Eigen vector as a string key. */
  bool CIORedis::get(SIORedis &arg_ds, const char* arg_key, Eigen::VectorXd &arg_vec)
  {
    // Get the key
    arg_ds.reply_ = (redisReply *)redisCommand(arg_ds.context_, "GET %s",arg_key);
    if(arg_ds.reply_->len <= 0){ freeReplyObject((void*)arg_ds.reply_); return false;  }

    // NOTE TODO : Probably faster to use sprintf.
    std::stringstream ss; ss<<arg_ds.reply_->str;
    for(int i=0;i<arg_vec.rows();++i) { ss>>arg_vec(i);  }
    freeReplyObject((void*)arg_ds.reply_);

    return true;
  }


  bool CIORedis::get(SIORedis &arg_ds, const char* arg_key, int &arg_int)
  {
    // Get the key
    arg_ds.reply_ = (redisReply *)redisCommand(arg_ds.context_, "GET %s",arg_key);
    if(arg_ds.reply_->len <= 0){ freeReplyObject((void*)arg_ds.reply_); return false;  }

    // NOTE TODO : Probably faster to use sprintf.
    std::stringstream ss; ss<<arg_ds.reply_->str;
    ss>>arg_int;
    freeReplyObject((void*)arg_ds.reply_);

    return true;
  }

} /* namespace scl */

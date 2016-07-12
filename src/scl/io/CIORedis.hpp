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
 * CIORedis.hpp
 *
 *  Created on: Jul 10, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */


#ifndef CIOREDIS_HPP_
#define CIOREDIS_HPP_

#include <Eigen/Core>
#include <hiredis/hiredis.h>

namespace scl
{
  /** Basic data for reading from and writing to a redis database...
   * Makes it easy to keep track of things..*/
  class SIORedis{
  public:
    redisContext *context_ = NULL;
    redisReply *reply_ = NULL;
    const char *hostname_ = "127.0.0.1";
    const int port_ = 6379;
    const timeval timeout_ = { 1, 500000 }; // 1.5 seconds

    // A scratch string for formatting messages
    char str_[1024];
  };

  /** A class to simplify IO operations using hiredis.
   *
   * For now, we'll have this class be stateless. It will
   * merely provide serialization/deserialization and wrap
   * the IO error checks into a coherent interface. Primarily
   * to avoid code replication. */
  class CIORedis
  {
  public:
    /** Connects to the Redis database using parameters specified in the
     * data structure. Also stores the connection details in the data
     * structure. */
    bool connect(SIORedis &arg_ds, bool arg_ping_server=false);

    /** Sends a message. */
    bool runCommand(SIORedis &arg_ds, char* arg_msg);

    /** Sets an Eigen vector as a string key. */
    bool set(SIORedis &arg_ds, const char* arg_key, const Eigen::VectorXd &arg_vec);

    /** Sets an Eigen vector as a string key. */
    bool get(SIORedis &arg_ds, const char* arg_key, Eigen::VectorXd &arg_vec);

    /** Default constructor. Does nothing */
    CIORedis() {}
    /** Default destructor. Does nothing */
    virtual ~CIORedis() {}
  };

} /* namespace scl */

#endif /* SRC_SCL_IO_CIOREDIS_HPP_ */

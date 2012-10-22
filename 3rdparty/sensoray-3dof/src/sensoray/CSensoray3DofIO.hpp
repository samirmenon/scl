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
 * \file CSensoray3DofIO.hpp
 *
 *  Created on: Oct 21, 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CSENSORAY3DOFIO_HPP_
#define CSENSORAY3DOFIO_HPP_

//For compiling the demo in cpp instead of c.
#ifdef __cplusplus
extern "C" {
#endif

  #include "app2600.h"    // Linux api to 2600 middleware

#ifdef __cplusplus
}
#endif

#include <string>

namespace sensoray
{
  /** A bunch of constants used by the Sensoray driver
   * to establish 3dof I/O */
  class SSensoray3DofIO
  {
  public:
    SSensoray3DofIO() :
      mm_ip_addr_("10.10.10.1"),
      mm_handle_(0),
      timeout_gateway_ms_(100),
      timeout_comport_ms_(100),
      retries_com_(50),
      retries_gateway_(50),
      s2620_channel_width_(0),
      s2620_channel_freq_(1),
      s2620_channel_pwm_(2),
      s2620_channel_encoder_(3),
      com_src_a_(LOGDEV_COM2),
      com_dest_a_(LOGDEV_COM1),
      com_baud_a_(SIO_BR_9600),
      com_src_b_(LOGDEV_COM4),
      com_dest_b_(LOGDEV_COM3),
      com_baud_b_(SIO_BR_115200),
      com_reject_ignore_(0),
      com_reject_evaluate_ (1)
    {}

    // CONSTANTS //////////////////////////////////////////////////////////////////
    /** Set this to the MM's IP address.*/
    const std::string mm_ip_addr_;
    /** This is the first MM in the system, so it is number 0.*/
    const int mm_handle_;

    /** This many milliseconds before timing out or retry gateway transactions.*/
    const int timeout_gateway_ms_;
    /** This many milliseconds before timing out or retry comport transactions.*/
    const int timeout_comport_ms_;

    /** Do up to this many comport retries. */
    const int retries_com_;
    /** Do up to this many gateway retries. */
    const int retries_gateway_;

    // 2620 channel usage for this app:
    /** Pulse width measurement.*/
    const int s2620_channel_width_;
    /** Frequency counter. */
    const int s2620_channel_freq_;
    /** Pulse width modulated output. */
    const int s2620_channel_pwm_;
    /** Incremental encoder input. */
    const int s2620_channel_encoder_;

    // Comport usage for this app.  With two null-modem cables, we can loop back two ports into two other ports:
    /** Transmit A. */
    const u8 com_src_a_;
    /** Receive A. */
    const u8 com_dest_a_;
    /** Baudrate for A. */
    const u16 com_baud_a_;

    /** Transmit B. */
    const u8 com_src_b_;
    /** Receive B. */
    const u8 com_dest_b_;
    /** Baudrate for B. */
    const u16 com_baud_b_;

    /** Ignore the comport REJ flag. */
    const int com_reject_ignore_;
    /** Treat comport REJ flag as an error. */
    const int com_reject_evaluate_;
  };

  /** This class provides a simple interface to connect
   * to a Sensoray board, read encoder positions and send
   * analog force/torque commands */
  class CSensoray3DofIO
  {
  public:
    CSensoray3DofIO();
    ~CSensoray3DofIO();
  };

} /* namespace sensoray */
#endif /* CSENSORAY3DOFIO_HPP_ */

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
 * \file CSensoray3DofIODriver.hpp
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
    /** Default constructor : Sets stuff to defaults */
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
      iters_ctrl_loop_(0),
      num_iom_boards_(0),
      iom_link_flags_(0),
      interlock_flags_(0)
    {//Somewhat redundant initialization to maintain declaration order
      int i;
      for (i=0; i<16; ++i)
      {
        iom_types_[i] = 0;
        iom_status_[i] = 0;
        s2608_num_aouts_at_iom_[i] = 0;
      }

      for (i=0; i<6; ++i)
      { num_digital_in_states_[i] = 0;  }

      for (i=0; i<16; ++i)
      { analog_in_voltages_[i] = 0;  }

      for (i=0; i<MAX_NUM_AOUTS; ++i)
      { analog_out_voltages_[i] = 0;  }

      for (i=0; i<4; ++i)
      {
        counter_counts_[i] = 0;
        counter_timestamp_[i] = 0;
      }

    }

    /** Default destructor : Does nothing */
    ~SSensoray3DofIO(){}

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

    // PUBLIC STORAGE ///////////////////////////////////////////////////////////////
    /** Number of times through the control loop so far. */
    int   iters_ctrl_loop_;

    /** Number of detected iom's. */
    u16   num_iom_boards_;
    /** Detected iom types. */
    u16   iom_types_[16];
    /** Iom status info. */
    u8    iom_status_[16];

    /** Number of dac channels (applies to 2608 only). */
    u8    s2608_num_aouts_at_iom_[16];

    // Input data from the i/o system.
    /** IOM port Link status. */
    u16   iom_link_flags_;
    /** Interlock power status. */
    u8    interlock_flags_;
    /** Digital input states (48 channels). */
    u8    num_digital_in_states_[6];
    /** Analog input voltages. */
    DOUBLE  analog_in_voltages_[16];

    // Output data to the i/o system.
    /** Analog output voltages. */
    DOUBLE  analog_out_voltages_[MAX_NUM_AOUTS];
    /** Counter data. */
    u32   counter_counts_[4];
    /** Counter timestamps. */
    u16   counter_timestamp_[4];
  };

  /** This class provides a simple interface to connect
   * to a Sensoray board, read encoder positions and send
   * analog force/torque commands */
  class CSensoray3DofIODriver
  {
  public:
    /** Default constructor : Does nothing */
    CSensoray3DofIODriver() :
      s_ds_(), max_main_modules_(1), max_io_modules_at_main_module_(16),
      totalSent(0), peek_character(-1) {}

    /** Default destructor : Does nothing */
    ~CSensoray3DofIODriver() {}

    /** Get data */
    SSensoray3DofIO& getData()
    { return s_ds_; }

    /** ***************** Driver calls (in order) *********************** */
    /** Initializes the main module.
     *
     * Assumptions :
     * 1. Only one main module in the system
     * 2. The main module's id is 0, and IP is 10.10.10.1
     * 3.  */
    bool init();

    /** Prints out an error message given an io module status
     * vector */
    void showErrorInfo( u32 gwerr, u8 *iom_status_ );

    int ioControlLoop( void );

    // Static fn. FORWARD REFERENCES ////////////////////////////////////////////////////////////

    void ioControlMain( void );

  private:
    /** The data structore with all the important vars etc. */
    SSensoray3DofIO s_ds_;

    const unsigned int max_main_modules_;
    const unsigned int max_io_modules_at_main_module_;

    int totalSent;
    int peek_character;
  };

} /* namespace sensoray */
#endif /* CSENSORAY3DOFIO_HPP_ */

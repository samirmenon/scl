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

#include "CSensoray3DofIODriver.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include <stdexcept>
#include <sstream>
#include <iostream>


namespace sensoray
{
  bool CSensoray3DofIODriver::init()
  {
    bool flag;
    try
    {
      u32 faults;

      // Open the 2600 main module. Assume max_main_modules_ == 1.
      faults = S26_DriverOpen( max_main_modules_ );
      if(0 != faults)
      {
        std::stringstream s;
        s<<"Could not open the driver. S26_DriverOpen() fault: "<<static_cast<int>(faults);
        throw(std::runtime_error(s.str()));
      }

      // Open the main module. Assume s_ds_.mm_handle_ == 0
      faults = S26_BoardOpen( s_ds_.mm_handle_, 0, s_ds_.mm_ip_addr_.c_str() );
      if(0 != faults)
      {
        std::stringstream s;
        s<<"Could not open connection to the board. S26_BoardOpen() fault: "<<static_cast<int>(faults);
        throw(std::runtime_error(s.str()));
      }

      // Reset the I/O system.
      flag = S26_ResetNetwork( s_ds_.mm_handle_ );
      if(false == flag)
      { throw(std::runtime_error("Could not reset the sensoray main and IO module network."));  }

      // Register all iom's.  If no errors, proceed to IO loop
      // Detect and register all sensoray I/O modules.
      faults = S26_RegisterAllIoms( s_ds_.mm_handle_, s_ds_.timeout_gateway_ms_,
          &s_ds_.num_iom_boards_, s_ds_.iom_types_, s_ds_.iom_status_, s_ds_.retries_gateway_ );
      if(0 != faults)
      {
        showErrorInfo( faults, s_ds_.iom_status_ );
        throw(std::runtime_error("Could register one or more sensoray IO modules."));
      }

      // Print all the registered I/O modules
      std::cout<<"\nDetected I/O modules on Sensoray Board:";
      for (unsigned int i = 0; i < max_io_modules_at_main_module_; i++ )
      {
        if ( 0 != s_ds_.iom_types_[i] )
        { std::cout<<"\nPort ["<<i<<"]. I/O module : "<<s_ds_.iom_types_[i]; }
      }

      // Initialize the 2608 (aout) and 2620 (din) IO modules
      // Doing so requires scheduling a transaction, with all the init loaded into it.
      void*   tran_hndl;        // Transaction object.

      // ========= Open Transaction =============
      // Start a new transaction. Create it using the sensoray driver API.
      tran_hndl = S26_SchedOpen( s_ds_.mm_handle_, s_ds_.retries_gateway_ );
      if (NULL == tran_hndl)
      { throw(std::runtime_error("S26_SchedOpen() : Could not allocate a transaction object." ));  }

      // Schedule the I/O actions into the transaction.
      // For each iom port on the MM ...
      for (unsigned int i = 0; i < max_io_modules_at_main_module_; i++ )
      {
        // Schedule some initialization actions based on iom type.
        switch ( s_ds_.iom_types_[i] )
        {
          case 2608:
            S26_Sched2608_ReadEeprom(tran_hndl, i, 0, &s_ds_.s2608_num_aouts_at_iom_[i] );  // Get the dac channel count.
            S26_Sched2608_SetLineFreq(tran_hndl, i, LINEFREQ_60HZ );   // 60 Hz line frequency (default).
            break;

          case 2620:
            // Set gate period to 1 second, timestamp resolution to 1 millisecond.
            S26_Sched2620_SetCommonControl( tran_hndl, i, 1000, 3 );

            // Set up encoder interface on s_ds_.s2620_channel_encoder_.
            S26_Sched2620_SetModeEncoder( tran_hndl, i, 0, 0, 0, 3 );
            S26_Sched2620_SetModeEncoder( tran_hndl, i, 1, 0, 0, 3 );
            S26_Sched2620_SetModeEncoder( tran_hndl, i, 2, 0, 0, 3 );
            S26_Sched2620_SetModeEncoder( tran_hndl, i, 3, 0, 0, 3 );

            break;
        }
      }

      // Execute the scheduled i/o and release the transaction object.
      faults = S26_SchedExecute( tran_hndl, s_ds_.timeout_gateway_ms_, s_ds_.iom_status_ );
      if (0 != faults)
      { //Report error if one was detected.
        showErrorInfo(faults, s_ds_.iom_status_);
        throw(std::runtime_error("Could execute transaction to initialize one or more sensoray IO modules."));
      }
      // ========= End Transaction =============

      return true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCSensoray3DofIODriver::init() : Error :"<<e.what(); }
    return false;
  }

  ///////////////////////////////////////////////////////
  // Main control loop.  Returns loop iteration count.

  int CSensoray3DofIODriver::ioControlLoop( void )
  {
    void*   x;        // Transaction object.
    u8    chan;
    struct  timeval tStart;
    struct  timeval tEnd;
    int       tDiff;
    int       tMax = 0;

    // Repeat until keypress ...
    for (int i=0 ;i<500 ;++i )
    {
      // Compute the next output states -----------------------------------------------------------
      // Bump analog output voltage images.
      for ( chan = 0; chan < MAX_NUM_AOUTS; chan++ )
        s_ds_.analog_out_voltages_[chan] = (double)( chan + 1 );

      // Schedule and execute the gateway I/O -----------------------------------------------------
      // Create a new transaction.
      x = S26_SchedOpen( s_ds_.mm_handle_, s_ds_.retries_gateway_ );

      // Report error if transaction couldn't be created.
      if ( x == 0 )
        printf( "Error: S26_SchedOpen() failed to allocate a transaction object.\n" );


      // Schedule all I/O into the transaction.
      int   chan;
      u8    i;

      // Schedule some 2601 actions.
#ifdef DEBUG
      S26_Sched2601_GetLinkStatus( x, &s_ds_.iom_link_flags_ );
      S26_Sched2601_GetInterlocks( x, &s_ds_.interlock_flags_ );
#endif

      // Schedule some iom actions.  For each iom port ...
      for ( i = 0; i < max_io_modules_at_main_module_; i++ )
      {
        // Schedule some i/o actions based on module type.
        switch( s_ds_.iom_types_[i] )
        {
          case 2608:
            // Update reference standards and read analog inputs
            S26_Sched2608_GetCalData( x, i, 0 );          // Auto-cal.  Only needed ~once/sec, but we always do it for simplicity.

            // Program all analog outputs.
            for ( chan = 0; chan < (int)s_ds_.s2608_num_aouts_at_iom_[chan]; chan++ )
              S26_Sched2608_SetAout( x, i, (u8)chan, s_ds_.analog_out_voltages_[chan] );
            break;

          case 2620:
            // Transfer counter cores to latches.
            S26_Sched2620_SetControlReg( x, 0, 0, 2 );
            S26_Sched2620_SetControlReg( x, 0, 1, 2 );
            S26_Sched2620_SetControlReg( x, 0, 2, 2 );

            // Read latches.
            for ( chan = 0; chan < 3; chan++ )
            { S26_Sched2620_GetCounts( x, i, (u8)chan, &s_ds_.counter_counts_[chan], &s_ds_.counter_timestamp_[chan] );  }// Fetch values from all channel latches.
            printf("\nEnc : %d %d %d", s_ds_.counter_counts_[0], s_ds_.counter_counts_[1], s_ds_.counter_counts_[2]);

            break;
        }
      }

      // Cache the i/o start time.
      gettimeofday( &tStart, 0 );

      // Execute the scheduled i/o and then release the transaction object.  Exit loop if there was no error.
      GWERR err;
      // Execute the scheduled i/o.  Report error if one was detected.
      if ( ( err = S26_SchedExecute( x, s_ds_.timeout_gateway_ms_, s_ds_.iom_status_ ) ) != 0 )
        showErrorInfo( err, s_ds_.iom_status_ );
      if ( (int)err )
      {
        printf( "Terminating i/o control loop.\n" );
        break;
      }

      // Compute elapsed time for the i/o.
      gettimeofday( &tEnd, 0 );
      tDiff = tEnd.tv_usec - tStart.tv_usec;
      if ( tDiff <= 0 )
        tDiff += 1000000;

      // Update worst-case i/o time.
      if ( tDiff > tMax )
        tMax = tDiff;

      // Bump the loop count.
      s_ds_.iters_ctrl_loop_++;
    }

    // Report worst-case i/o time.
    printf( "Worst-case I/O time (msec):  %d\n", tMax / 1000 );

    // Return cycle count.
    return s_ds_.iters_ctrl_loop_;
  }

  /////////////////////////////////////////////////////////
  // Initialize all I/O and run control loop "forever."

  void CSensoray3DofIODriver::ioControlMain( void )
  {
    u8    i;
    int   j;
    void*   x;        // Transaction object.
    time_t  StartTime;    // Benchmark start time.
    double  tElapsed;   // Benchmark elapsed time.

    // MAIN CONTROL LOOP ====================================================================================

    printf( "\nRunning main control loop\nHit any key to terminate\n" );

    // Start benchmark timer.
    StartTime = time( NULL );

    // Run control loop until terminated or error.
    s_ds_.iters_ctrl_loop_ = ioControlLoop();

    // Stop benchmark timer.
    tElapsed = difftime( time( NULL ), StartTime );

    // Show the final system state ============================================================================

    // For each iom port ...
    for ( i = 0; i < max_io_modules_at_main_module_; i++ )
    {
      switch ( s_ds_.iom_types_[i] )
      {
        case 2620:
          // Print the dio input states.
          printf( "Counter states:" );
          for ( j = 0; j < 4; j++ )
            printf( "%d:%8.8lX ", j, s_ds_.counter_counts_[j] );
          printf( "\n" );
          break;
      }
    }

    // Report benchmark results.
    printf( "Control loop cycles:    %d\n", s_ds_.iters_ctrl_loop_ );
    printf( "Elapsed time (seconds): %lu\n", (u32)tElapsed );
    printf( "Average I/O cycle time (msec):  %.2f\n", tElapsed / (double)s_ds_.iters_ctrl_loop_ * 1000.0 );
  }

  ////////////////////////////////
  // Display gateway error info.
  void CSensoray3DofIODriver::showErrorInfo( u32 gwerr, u8 *arg_iom_status )
  {
    char  errmsg[128];
    int   ExtraInfo = gwerr & ~GWERRMASK;
    u8    status;

    switch ( gwerr & GWERRMASK )
    {
      case GWERR_IOMSPECIFIC:

        status = arg_iom_status[ExtraInfo];

        sprintf( errmsg, "Iom-specific error on iomport %d", ExtraInfo );

        if ( arg_iom_status )
        {
          switch( s_ds_.iom_types_[ExtraInfo] )
          {
            case 2608:
              if ( status & STATUS_2608_CALERR )  strcat( errmsg, ": using default cal values" );
              break;

            case 2610:
              if ( status & STATUS_2610_STRM )  strcat( errmsg, ": serial stream error" );
              break;

            case 2650:
              if ( status & STATUS_2650_DRVR )  strcat( errmsg, ": relay coil driver fault" );
              if ( status & STATUS_2650_STRM )  strcat( errmsg, ": serial stream error" );
              break;
            case 2652:
              if ( status & STATUS_2652_STRM )  strcat( errmsg, ": serial stream error" );
              break;
          }
        }

        break;

      case GWERR_BADVALUE:    sprintf( errmsg, "Illegal value for argument %d", ExtraInfo );        break;
      case GWERR_IOMCLOSED:   sprintf( errmsg, "Iom is not open" );                       break;
      case GWERR_IOMERROR:    sprintf( errmsg, "Iom CERR asserted" );                       break;
      case GWERR_IOMNORESPOND:  sprintf( errmsg, "Bad module response from iomport %d", ExtraInfo );    break;
      case GWERR_IOMRESET:    sprintf( errmsg, "Module RST asserted" );                     break;
      case GWERR_IOMTYPE:     sprintf( errmsg, "Action not supported by iom type" );                break;
      case GWERR_MMCLOSED:    sprintf( errmsg, "MM is closed" );                          break;
      case GWERR_MMNORESPOND:   sprintf( errmsg, "MM response timed out" );                     break;
      case GWERR_PACKETSEND:    sprintf( errmsg, "Failed to send cmd packet" );                   break;
      case GWERR_TOOLARGE:    sprintf( errmsg, "Command or response packet too large" );              break;
      case GWERR_XACTALLOC:   sprintf( errmsg, "Transaction Object allocation problem" );             break;
      default:          sprintf( errmsg, "Unknown error" );                         break;
    }

    printf( "Error: 0x%X (%s), s_ds_.iters_ctrl_loop_ = %d.\n", (int)gwerr, errmsg, s_ds_.iters_ctrl_loop_ );
  }

} /* namespace sensoray */

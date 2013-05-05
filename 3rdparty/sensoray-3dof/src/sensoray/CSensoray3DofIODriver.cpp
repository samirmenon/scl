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
  CSensoray3DofIODriver::CSensoray3DofIODriver() :
            s_ds_(),
            mode_encoder_only_(false),
            max_main_modules_(1),
            max_io_modules_at_main_module_(2),
            enc_mm_id_(0),
            dac_mm_id_(1),
            sensoray_calibrate_ctr(0)
  { }

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

      // ========= Open Transaction : INITIALIZE THE SENSORAY IO MODULES =============
      // Initialize the 2608 (aout) and 2620 (din) IO modules
      // Doing so requires scheduling a transaction, with all the init loaded into it.
      void*   tran_hndl;        // Transaction object.

      // Start a new transaction. Create it using the sensoray driver API.
      tran_hndl = S26_SchedOpen( s_ds_.mm_handle_, s_ds_.retries_gateway_ );
      if (NULL == tran_hndl)
      { throw(std::runtime_error("S26_SchedOpen() : Could not allocate a transaction object." ));  }

      // Schedule the I/O actions into the transaction.
      if(2620 != s_ds_.iom_types_[0])
      { throw(std::runtime_error("Incorrect setup. Please connect 2620 to Main Module port 0.")); }
      else
      {// Case 2620: Set up encoder interface on the first three ports
        S26_Sched2620_SetModeEncoder( tran_hndl, enc_mm_id_, 0, 0, 0, 3 );
        S26_Sched2620_SetModeEncoder( tran_hndl, enc_mm_id_, 1, 0, 0, 3 );
        S26_Sched2620_SetModeEncoder( tran_hndl, enc_mm_id_, 2, 0, 0, 3 );
      }

      if(2608 != s_ds_.iom_types_[1])
      {
        std::cout<<"\n WARNING : Encoder only mode. 2608 not connected to Main Module port 1.";
        mode_encoder_only_ = true;
      }
      else
      {// Case 2608: Count the number of dac output channels and set the line frequency
        S26_Sched2608_ReadEeprom(tran_hndl, dac_mm_id_, 0, &s_ds_.s2608_num_aouts_at_iom_ );  // Get the dac channel count.
        S26_Sched2608_SetLineFreq(tran_hndl, dac_mm_id_, LINEFREQ_60HZ );   // 60 Hz line frequency (default).
        mode_encoder_only_ = false;
      }

      // Execute the scheduled i/o and release the transaction object.
      faults = S26_SchedExecute( tran_hndl, s_ds_.timeout_gateway_ms_, s_ds_.iom_status_ );
      if (0 != faults)
      { //Report error if one was detected.
        showErrorInfo(faults, s_ds_.iom_status_);
        throw(std::runtime_error("Could execute transaction to initialize one or more sensoray IO modules."));
      }
      // ========= End Transaction : INITIALIZE THE SENSORAY IO MODULES =============

      //Error checks: If motors are to be controlled, there should be 3 dac outputs
      if( (false == mode_encoder_only_) && (3 != s_ds_.s2608_num_aouts_at_iom_ ) )
      {std::cout<<"\n WARNING : Did not find 3 analog outs. A outs = "<<s_ds_.s2608_num_aouts_at_iom_; }

      std::cout<<"\nInitalized Main module, Encoder in (2620), and Analog out (2608)"<<std::flush;

      sensoray_calibrate_ctr = 0;

      return true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCSensoray3DofIODriver::init() : Error :"<<e.what(); }
    return false;
  }

  /** Closes the driver and shuts down the modules */
  void CSensoray3DofIODriver::shutdown()
  {
    long c0, c1, c2;
    bool flag = readEncodersAndCommandMotors(c0, c1, c2, 0.0, 0.0, 0.0);
    if(false == flag)
    { std::cout<<"\nError : Could not reset the motors to zero during shutdown!"; }
    S26_DriverClose();
  }

  /** Encoder operation only : Reads encoders */
  bool CSensoray3DofIODriver::readEncoders(
      long& c0, long& c1, long& c2)
  {
    u32 tc0, tc1, tc2; //Sensoray data types for reading encoders

    //Open transaction
    void* tran_hndl = S26_SchedOpen( s_ds_.mm_handle_, s_ds_.retries_gateway_ );
    if(NULL == tran_hndl)
    { return false; }

    // Transfer counter cores to latches.
    S26_Sched2620_SetControlReg( tran_hndl, enc_mm_id_, 0, 2 );
    S26_Sched2620_SetControlReg( tran_hndl, enc_mm_id_, 1, 2 );
    S26_Sched2620_SetControlReg( tran_hndl, enc_mm_id_, 2, 2 );

    // Read latches.
    S26_Sched2620_GetCounts( tran_hndl, enc_mm_id_, 0, &tc0, 0);
    S26_Sched2620_GetCounts( tran_hndl, enc_mm_id_, 1, &tc1, 0);
    S26_Sched2620_GetCounts( tran_hndl, enc_mm_id_, 2, &tc2, 0);

    // Execute the scheduled i/o and then release the transaction object.  Return false if there was an error.
    // We don't care about the I/O module status, so last arg is zero
    GWERR err = S26_SchedExecute(tran_hndl, s_ds_.timeout_gateway_ms_, 0);
    if (0 != err)
    {
      showErrorInfo( err, s_ds_.iom_status_ );
      return false;
    }

    //Transaction exectued. We now have the values.
    c0 = tc0;
    c1 = tc1;
    c2 = tc2;

    return true;
  }

  /** Encoder+Motor operation : Sends analog out to motors + reads encoders */
  bool CSensoray3DofIODriver::readEncodersAndCommandMotors(long& c0, long& c1, long& c2,
      const double m0, const double m1, const double m2)
  {
    if(mode_encoder_only_) //Can't operate motors just yet
    { return false; }

    //Set the motor control commands
    s_ds_.analog_out_voltages_[0] = m0;
    s_ds_.analog_out_voltages_[1] = m1;
    s_ds_.analog_out_voltages_[2] = m2;

    //Open transaction
    void* tran_hndl = S26_SchedOpen( s_ds_.mm_handle_, s_ds_.retries_gateway_ );
    if(NULL == tran_hndl)
    { return false; }

    // Transfer counter cores to latches.
    S26_Sched2620_SetControlReg( tran_hndl, enc_mm_id_, 0, 2 );
    S26_Sched2620_SetControlReg( tran_hndl, enc_mm_id_, 1, 2 );
    S26_Sched2620_SetControlReg( tran_hndl, enc_mm_id_, 2, 2 );

    // Read latches.
    S26_Sched2620_GetCounts( tran_hndl, enc_mm_id_, 0, &s_ds_.counter_counts_[0], 0);
    S26_Sched2620_GetCounts( tran_hndl, enc_mm_id_, 1, &s_ds_.counter_counts_[1], 0);
    S26_Sched2620_GetCounts( tran_hndl, enc_mm_id_, 2, &s_ds_.counter_counts_[2], 0);

    //Init motors
    // Update reference standards and read analog inputs
    // Auto-cal.  Only needed ~once/sec, but we do every 50 loops (a few times/sec).
#ifdef DEBUG
    S26_Sched2608_GetCalData( tran_hndl, dac_mm_id_, 0 );          // In debug mode, we always do it for simplicity.
#else
    if(sensoray_calibrate_ctr==0)
    { S26_Sched2608_GetCalData(tran_hndl, dac_mm_id_, 0 ); }
    sensoray_calibrate_ctr++;
    if(sensoray_calibrate_ctr>150) { sensoray_calibrate_ctr = 0; }
#endif

    // Program all analog outputs.
    S26_Sched2608_SetAout(tran_hndl, dac_mm_id_, 0, s_ds_.analog_out_voltages_[0] );
    S26_Sched2608_SetAout(tran_hndl, dac_mm_id_, 1, s_ds_.analog_out_voltages_[1] );
    S26_Sched2608_SetAout(tran_hndl, dac_mm_id_, 2, s_ds_.analog_out_voltages_[2] );

    // Execute the scheduled i/o and then release the transaction object.  Return false if there was an error.
    // We don't care about the I/O module status, so last arg is zero
    GWERR err = S26_SchedExecute(tran_hndl, s_ds_.timeout_gateway_ms_, 0);
    if (0 != err)
    {
      showErrorInfo( err, s_ds_.iom_status_ );
      return false;
    }

    //Return the values
    c0 = s_ds_.counter_counts_[0];
    c1 = s_ds_.counter_counts_[1];
    c2 = s_ds_.counter_counts_[2];

    return true;
  }

  ///////////////////////////////////////////////////////
  // Main control loop.  Returns loop iteration count.

  bool CSensoray3DofIODriver::testDriver(const unsigned int arg_control_loops, const double arg_motor_voltage)
  {
    void*   tran_hndl;        // Transaction object.
    struct  timeval tStart;
    struct  timeval tEnd;
    int       tDiff;
    int       tMax = 0;

    for (unsigned int i=0; i<arg_control_loops; ++i)
    {
      // Compute the next output states -----------------------------------------------------------
      // Bump analog output voltage images.
      for (unsigned int chan = 0; chan < MAX_NUM_AOUTS; chan++ )
      { s_ds_.analog_out_voltages_[chan] = arg_motor_voltage;  }

      // Schedule and execute the gateway I/O -----------------------------------------------------
      // Create a new transaction.
      tran_hndl = S26_SchedOpen( s_ds_.mm_handle_, s_ds_.retries_gateway_ );

      // Report error if transaction couldn't be created.
      if ( tran_hndl == 0 )
      {  printf( "Error: S26_SchedOpen() failed to allocate a transaction object.\n" );  }

      // Schedule some 2601 actions.
#ifdef DEBUG
      //NOTE TODO : These should have associated checks.
      S26_Sched2601_GetLinkStatus( tran_hndl, &s_ds_.iom_link_flags_ );
      S26_Sched2601_GetInterlocks( tran_hndl, &s_ds_.interlock_flags_ );
#endif

      // Schedule some iom actions.  For each iom port ...
      for (unsigned int j = 0; j < max_io_modules_at_main_module_; j++ )
      {
        // Schedule some i/o actions based on module type.
        switch( s_ds_.iom_types_[j] )
        {
          case 2608:
            // Update reference standards and read analog inputs
            S26_Sched2608_GetCalData( tran_hndl,j, 0 );          // Auto-cal.  Only needed ~once/sec, but we always do it for simplicity.
            // Program all analog outputs.
            for (unsigned int chan = 0; chan < (int)s_ds_.s2608_num_aouts_at_iom_; chan++ )
            {
            	if(i==arg_control_loops-1)
            	{ s_ds_.analog_out_voltages_[chan] = 0; }
            	S26_Sched2608_SetAout( tran_hndl, j, (u8)chan, s_ds_.analog_out_voltages_[chan] );
            }
            break;

          case 2620:
            // Transfer counter cores to latches.
            S26_Sched2620_SetControlReg( tran_hndl, 0, 0, 2 );
            S26_Sched2620_SetControlReg( tran_hndl, 0, 1, 2 );
            S26_Sched2620_SetControlReg( tran_hndl, 0, 2, 2 );

            // Read latches.
            for (unsigned int chan = 0; chan < 3; chan++ )
            { S26_Sched2620_GetCounts( tran_hndl, j, (u8)chan, &s_ds_.counter_counts_[chan], &s_ds_.counter_timestamp_[chan] );  }// Fetch values from all channel latches.
            printf("\nEnc : %ld %ld %ld", s_ds_.counter_counts_[0], s_ds_.counter_counts_[1], s_ds_.counter_counts_[2]);

            break;
        }
      }

      // Cache the i/o start time.
      gettimeofday( &tStart, 0 );

      // Execute the scheduled i/o and then release the transaction object.  Exit loop if there was no error.
      GWERR err;
      // Execute the scheduled i/o.  Report error if one was detected.
      err = S26_SchedExecute( tran_hndl, s_ds_.timeout_gateway_ms_, s_ds_.iom_status_ );
      if (0 != err)
      {
        showErrorInfo( err, s_ds_.iom_status_ );
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

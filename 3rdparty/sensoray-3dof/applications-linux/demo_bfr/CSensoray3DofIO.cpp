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

#include "CSensoray3DofIO.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

namespace sensoray
{
  ////////////////////////////////
  // Display gateway error info.

  void CSensoray3DofIO::ShowErrorInfo( u32 gwerr, u8 *arg_iom_status )
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

  //////////////////////////////////////
  // Process comport error, if any.

  u32 CSensoray3DofIO::ComError( u32 gwerr, const char *fname, int evalComReject )
  {
    char errmsg[128];

    // If transaction error was detected ...
    if ( gwerr & GWERRMASK )
    {
      switch ( gwerr & GWERRMASK )
      {
        case GWERR_BADVALUE:    sprintf( errmsg, "Illegal value for argument %lu", gwerr & ~GWERRMASK );  break;
        case GWERR_IOMRESET:    sprintf( errmsg, "Module RST asserted" );                 break;
        case GWERR_MMCLOSED:    sprintf( errmsg, "MM is closed" );                      break;
        case GWERR_MMNORESPOND:   sprintf( errmsg, "MM response timed out" );                 break;
        case GWERR_PACKETSEND:    sprintf( errmsg, "Failed to send cmd packet" );               break;
        case GWERR_TOOLARGE:    sprintf( errmsg, "Command or response packet too large" );          break;
        case GWERR_XACTALLOC:   sprintf( errmsg, "Transaction Object allocation problem" );         break;
        default:          sprintf( errmsg, "Unknown error" );                     break;
      }
    }

    // If comport error was detected ...
    else if ( gwerr & COM_FRAMINGERROR )  sprintf( errmsg, "Framing error" );
    else if ( gwerr & COM_PARITYERROR )   sprintf( errmsg, "Parity error" );
    else if ( gwerr & COM_OVERFLOWERROR ) sprintf( errmsg, "Receiver overflow" );

    // If comport command was rejected and checking for this condition is enabled ...
    else if ( ( ( gwerr & COM_REJECTED ) != 0 ) && ( evalComReject == s_ds_.com_reject_evaluate_ ) )
      sprintf( errmsg, "Command rejected" );

    // If no errors were detected ...
    else
      return GWERR_NONE;

    printf( "%s() error: 0x%lX (%s), s_ds_.iters_ctrl_loop_ = %d.\n", fname, gwerr, errmsg, s_ds_.iters_ctrl_loop_ );
    return gwerr;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Detect and register all i/o modules connected to the 2601 main module.

  int CSensoray3DofIO::DetectAllIoms( void )
  {
    int i;
    u32 faults;

    // Detect and register all iom's.
    if ( ( faults = S26_RegisterAllIoms( s_ds_.mm_handle_, s_ds_.timeout_gateway_ms_,
        &s_ds_.num_iom_boards_, s_ds_.iom_types_, s_ds_.iom_status_, s_ds_.retries_gateway_ ) ) != 0 )
    {
      ShowErrorInfo( faults, s_ds_.iom_status_ );
      return 0; // failed.
    }

    // List the discovered iom's.
    printf( "DETECTED IOM'S:\n" );
    for ( i = 0; i < 16; i++ )
    {
      if ( s_ds_.iom_types_[i] )
        printf( " port %2.2d: %4.4d\n", i, s_ds_.iom_types_[i] );
    }

    return 1; // success.
  }

  ///////////////////////////////////////////////////////////////
  // Schedule some gateway I/O into transaction object x
  void CSensoray3DofIO::sched_io( void* x )
  {
    int   chan;
    u8    i;

    // Schedule some 2601 actions.
    S26_Sched2601_GetLinkStatus( x, &s_ds_.iom_link_flags_ );
    S26_Sched2601_GetInterlocks( x, &s_ds_.interlock_flags_ );

    // Schedule some iom actions.  For each iom port ...
    for ( i = 0; i < 16; i++ )
    {
      // Schedule some i/o actions based on module type.
      switch( s_ds_.iom_types_[i] )
      {
        case 2608:
          // Update reference standards and read analog inputs
          S26_Sched2608_GetCalData( x, i, 0 );          // Auto-cal.  Only needed ~once/sec, but we always do it for simplicity.
          S26_Sched2608_GetAins( x, i, s_ds_.analog_in_voltages_, ADC_INTEGRATED ); // Fetch the analog inputs.

          // Program all analog outputs.
          for ( chan = 0; chan < (int)s_ds_.num_2608_aouts_at_iom_[chan]; chan++ )
            S26_Sched2608_SetAout( x, i, (u8)chan, s_ds_.analog_out_voltages_[chan] );
          break;

        case 2610:
          S26_Sched2610_SetOutputs( x, i, s_ds_.num_digital_out_states_ );    // Program the dio outputs.
          S26_Sched2610_GetInputs( x, i, s_ds_.num_digital_in_states_ );      // Fetch physical inputs.
          break;

        case 2620:
          // Transfer counter cores to latches.
          S26_Sched2620_SetControlReg( x, i, s_ds_.s2620_channel_pwm_, 2 );
          S26_Sched2620_SetControlReg( x, i, s_ds_.s2620_channel_encoder_, 2 );

          // Read latches.
          for ( chan = 0; chan < 4; chan++ )
            S26_Sched2620_GetCounts( x, i, (u8)chan, &s_ds_.counter_counts_[chan], &s_ds_.counter_timestamp_[chan] ); // Fetch values from all channel latches.
          break;

        case 2650:
          S26_Sched2650_SetOutputs( x, i, &s_ds_.num_relay_states_ );   // Program the relays.
          break;
        case 2652:
          S26_Sched2652_SetOutputs( x, i, &s_ds_.num_relay_states_ );   // Program the relays.
          break;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialize two serial comports, one of which may be looped back into the other via a null-modem cable.
  // Returns zero if successful.

  int CSensoray3DofIO::SerialInit( u8 ComSrc, u8 ComDst, u16 BaudRate )
  {
    u32 comstatus;

    // Source (transmitter) port.
    comstatus = S26_ComSetMode( s_ds_.mm_handle_,
        ComSrc,
        BaudRate,
        SIO_PHY_RS232 | SIO_PARITY_NONE | SIO_DATA_8 | SIO_STOP_1 | SIO_FLOW_OFF,
        SIO_LED_TRANSMIT,
        s_ds_.timeout_comport_ms_,
        s_ds_.retries_com_ );
    if ( ComError( comstatus, "S26_ComSetMode", s_ds_.com_reject_evaluate_ ) )
      return 1;

    comstatus = S26_ComOpen( s_ds_.mm_handle_, ComSrc, s_ds_.timeout_comport_ms_, s_ds_.retries_com_ );
    if ( ComError( comstatus, "S26_ComOpen", s_ds_.com_reject_evaluate_ ) )
      return 1;

    // Destination (receiver) port.
    comstatus = S26_ComSetMode( s_ds_.mm_handle_,
        ComDst,
        BaudRate,
        SIO_PHY_RS232 | SIO_PARITY_NONE | SIO_DATA_8 | SIO_STOP_1 | SIO_FLOW_OFF,
        SIO_LED_RECEIVE,
        s_ds_.timeout_comport_ms_,
        s_ds_.retries_com_ );
    if ( ComError( comstatus, "S26_ComSetMode", s_ds_.com_reject_evaluate_ ) )
      return 1;

    comstatus = S26_ComOpen( s_ds_.mm_handle_, ComDst, s_ds_.timeout_comport_ms_, s_ds_.retries_com_ );
    if ( ComError( comstatus, "S26_ComOpen", s_ds_.com_reject_evaluate_ ) )
      return 1;

    return 0;
  }

  //////////////////////////////////////////////
  // Do some serial I/O.
  // Returns zero if successful.

  int CSensoray3DofIO::SerialIo( u8 ComSrc, u8 ComDst )
  {
    int   msglen;
    char  SndBuf[1024];   // String to be sent.
    char  RcvBuf[1024];   // Buffer that will receive the string.
    u16   BufLen;       // Max number of characters to receive.
    u32   comstatus;

    // Construct a message.
    sprintf( SndBuf, "12345678901234567890123456:%d\n", s_ds_.iters_ctrl_loop_ );
    msglen = strlen( SndBuf );

    // Attempt to send message.  Ignore the COM_REJECT error, which is caused by insufficient transmit buffer free space.
    comstatus = S26_ComSend( s_ds_.mm_handle_, ComSrc, (u8 *)SndBuf, (u16)msglen, s_ds_.timeout_comport_ms_, s_ds_.retries_com_ );
    if ( ComError( comstatus, "S26_ComSend", s_ds_.com_reject_ignore_ ) )
    {
      printf( "TotalSent = %d\n", totalSent );
      return 1;
    }
    else
      totalSent += msglen;

    // Receive all available data from the receiving comport's receive buffer.
    BufLen = sizeof(RcvBuf);     // Max number of characters to receive.
    comstatus = S26_ComReceive( s_ds_.mm_handle_, ComDst, (u8 *)RcvBuf, &BufLen, s_ds_.timeout_comport_ms_, s_ds_.retries_com_ );
    if ( ComError( comstatus, "S26_ComReceive", s_ds_.com_reject_evaluate_ ) )
    {
      printf( "TotalSent = %d\n", totalSent );
      return 1;
    }
    else
      RcvBuf[BufLen] = 0;           // Append null to end of string.

    // Success.
    return 0;
  }

  ///////////////////////////////////////////////////////
  // Main control loop.  Returns loop iteration count.

  int CSensoray3DofIO::io_control_loop( void )
  {
    void*   x;        // Transaction object.
    u8    chan;
    struct  timeval tStart;
    struct  timeval tEnd;
    int       tDiff;
    int       tMax = 0;

    // Repeat until keypress ...
    for ( ; ; )
    {
      // Wait for a real-time tick (crude simulation via Sleep used here).
      // Disable this to benchmark the i/o system performance.
      //    Sleep( 100 );

      // Compute the next output states -----------------------------------------------------------

      // Bump the relay state images.
      s_ds_.num_relay_states_++;

      // Bump the dio state images.
      if ( ++s_ds_.num_digital_out_states_[0] == 0 )
        if ( ++s_ds_.num_digital_out_states_[1] == 0 )
          if ( ++s_ds_.num_digital_out_states_[2] == 0 )
            if ( ++s_ds_.num_digital_out_states_[3] == 0 )
              if ( ++s_ds_.num_digital_out_states_[4] == 0 )
                ++s_ds_.num_digital_out_states_[5];

      // Bump analog output voltage images.
      for ( chan = 0; chan < MAX_NUM_AOUTS; chan++ )
        s_ds_.analog_out_voltages_[chan] = (double)( chan + 1 );

      // Schedule and execute the gateway I/O --------------------------------------------------------------

      // First do some serial io.
      if ( SerialIo( s_ds_.com_src_a_, s_ds_.com_dest_a_ ) )
        return s_ds_.iters_ctrl_loop_;

      if ( SerialIo( s_ds_.com_src_b_, s_ds_.com_dest_b_ ) )
        return s_ds_.iters_ctrl_loop_;

      // Start a new transaction.
      x = CreateTransaction( s_ds_.mm_handle_ );

      // Schedule all I/O into the transaction.
      sched_io( x );

      // Cache the i/o start time.
      gettimeofday( &tStart, 0 );

      // Execute the scheduled i/o and then release the transaction object.  Exit loop if there was no error.
      if ( io_exec( x ) )
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

      // Exit on keypress.
      if ( kbhit() )
      {
        kbread(); // Flush the keypress.
        break;
      }
    }

    // Report worst-case i/o time.
    printf( "Worst-case I/O time (msec):  %d\n", tMax / 1000 );

    // Return cycle count.
    return s_ds_.iters_ctrl_loop_;
  }

  /////////////////////////////////////////////////////////
  // Initialize all I/O and run control loop "forever."

  void CSensoray3DofIO::io_control_main( void )
  {
    u8    i;
    int   j;
    void*   x;        // Transaction object.
    time_t  StartTime;    // Benchmark start time.
    double  tElapsed;   // Benchmark elapsed time.

    // Analog input types for the 2608 iom.  These can be any supported voltage or thermocouple types.
    // Voltage types return volts, temperature types return degrees C.
    const u8 ain_types[] = {
        V_10_TYPE,  // chan 0: 10V range on channels 0-15.
        V_10_TYPE,  // chan 1
        V_10_TYPE,  // chan 2
        V_10_TYPE,  // chan 3
        V_10_TYPE,  // chan 4
        V_10_TYPE,  // chan 1
        V_10_TYPE,  // chan 6
        V_10_TYPE,  // chan 7
        V_10_TYPE,  // chan 8
        V_10_TYPE,  // chan 9
        V_10_TYPE,  // chan 10
        V_10_TYPE,  // chan 11
        V_10_TYPE,  // chan 12
        V_10_TYPE,  // chan 13
        V_10_TYPE,  // chan 14
        V_10_TYPE // chan 15
    };

    // Initialize serial comports.
    SerialInit( s_ds_.com_src_a_, s_ds_.com_dest_a_, s_ds_.com_baud_a_ );
    SerialInit( s_ds_.com_src_b_, s_ds_.com_dest_b_, s_ds_.com_baud_b_ );

    // INITIALIZE IOM's ================================================================================

    // Start a new transaction.
    x = CreateTransaction( s_ds_.mm_handle_ );

    // Schedule the I/O actions into the transaction.
    // For each iom port on the MM ...
    for ( i = 0; i < 16; i++ )
    {
      // Schedule some initialization actions based on iom type.
      switch ( s_ds_.iom_types_[i] )
      {
        case 2608:

          S26_Sched2608_ReadEeprom( x, i, 0, &s_ds_.num_2608_aouts_at_iom_[i] );  // Get the dac channel count.
          S26_Sched2608_SetLineFreq( x, i, LINEFREQ_60HZ );   // 60 Hz line frequency (default).
          S26_Sched2608_SetAinTypes( x, i, ain_types );     // Declare the analog input types.

          break;

        case 2620:

          // Set gate period to 1 second, timestamp resolution to 1 millisecond.
          S26_Sched2620_SetCommonControl( x, i, 1000, 3 );

          // Set up the pwm generator on s_ds_.s2620_channel_pwm_.
          S26_Sched2620_SetPreload( x, i, s_ds_.s2620_channel_pwm_, 1, 99 );    // Preload 1: on time.
          S26_Sched2620_SetPreload( x, i, s_ds_.s2620_channel_pwm_, 0, 4899 );    // Preload 0: off time.
          S26_Sched2620_SetModePwmGen( x, i, s_ds_.s2620_channel_pwm_, 0 );   // Configure as pwm generator.

          // Set up the frequency counter on s_ds_.s2620_channel_freq_.
          S26_Sched2620_SetModeFreqMeas( x, i, s_ds_.s2620_channel_freq_, 1 );    // Configure channel 1: freq counter, internal gate.

          // Set up for pulse width measurement on CHAN_PULSE.
          S26_Sched2620_SetModePulseMeas( x, i, s_ds_.s2620_channel_width_, 0 );    // active high input.

          // Set up encoder interface on s_ds_.s2620_channel_encoder_.
          S26_Sched2620_SetModeEncoder( x, i, s_ds_.s2620_channel_encoder_, 0, 0, 3 );

          break;
      }
    }

    // Execute the scheduled i/o and release the transaction object.
    if ( io_exec( x ) )
      return;

    // MAIN CONTROL LOOP ====================================================================================

    printf( "Running main control loop\nHit any key to terminate\n" );

    // Start benchmark timer.
    StartTime = time( NULL );

    // Run control loop until terminated or error.
    s_ds_.iters_ctrl_loop_ = io_control_loop();

    // Stop benchmark timer.
    tElapsed = difftime( time( NULL ), StartTime );

    // Show the final system state ============================================================================

    // For each iom port ...
    for ( i = 0; i < 16; i++ )
    {
      switch ( s_ds_.iom_types_[i] )
      {
        case 2608:
          // Print the analog inputs.
          printf( "AIN states:\n" );
          for ( j = 0; j < 16; j++ )
            printf( " %8f\n", s_ds_.analog_in_voltages_[j] );
          break;

        case 2610:
          // Print the dio input states.
          printf( "DIN states:" );
          for ( j = 0; j < 6; j++ )
            printf( " %2.2hhX", s_ds_.num_digital_in_states_[j] );
          printf( "\n" );
          break;

        case 2620:
          // Print the dio input states.
          printf( "Counter states:" );
          for ( j = 0; j < 4; j++ )
            printf( "%d:%8.8lX ", j, s_ds_.counter_counts_[j] );
          printf( "\n" );
          break;

        case 2650:
          break;

        case 2652:
          break;
      }
    }

    // Report benchmark results.
    printf( "Control loop cycles:    %d\n", s_ds_.iters_ctrl_loop_ );
    printf( "Elapsed time (seconds): %lu\n", (u32)tElapsed );
    printf( "Average I/O cycle time (msec):  %.2f\n", tElapsed / (double)s_ds_.iters_ctrl_loop_ * 1000.0 );
  }

  ////////////////////////////////////////////////////////
  // Start a new transaction.
  // Returns non-zero transaction handle if successful.

  void* CSensoray3DofIO::CreateTransaction( HBD hbd )
  {
    // Create a new transaction.
    void* x = S26_SchedOpen( hbd, s_ds_.retries_gateway_ );

    // Report error if transaction couldn't be created.
    if ( x == 0 )
      printf( "Error: S26_SchedOpen() failed to allocate a transaction object.\n" );

    return x;
  }

  ////////////////////////////////
  // Execute all scheduled i/o.
  // Returns zero if successful.

  int CSensoray3DofIO::io_exec( void* x )
  {
    GWERR err;

    // Execute the scheduled i/o.  Report error if one was detected.
    if ( ( err = S26_SchedExecute( x, s_ds_.timeout_gateway_ms_, s_ds_.iom_status_ ) ) != 0 )
      ShowErrorInfo( err, s_ds_.iom_status_ );

    return (int)err;
  }

  ////////////////////////////////////////////////
  // Keyboard functions.



  void CSensoray3DofIO::kbopen( void )
  {
    tcgetattr( 0, &initial_settings );
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr( 0, TCSANOW, &new_settings );
  }

  void CSensoray3DofIO::kbclose( void )
  {
    tcsetattr(0, TCSANOW, &initial_settings);
  }

  int CSensoray3DofIO::kbhit( void )
  {
    unsigned char ch;
    int nread;

    if ( peek_character != -1 )
      return 1;

    new_settings.c_cc[VMIN] = 0;
    tcsetattr( 0, TCSANOW, &new_settings );

    nread = read( 0, &ch, 1 );

    new_settings.c_cc[VMIN] = 1;
    tcsetattr( 0, TCSANOW, &new_settings );

    if ( nread == 1 )
    {
      peek_character = ch;
      return 1;
    }

    return 0;
  }

  int CSensoray3DofIO::kbread( void )
  {
    char ch;

    if ( peek_character != -1 )
    {
      ch = peek_character;
      peek_character = -1;
      return ch;
    }

    read( 0, &ch, 1 );

    return ch;
  }

} /* namespace sensoray */

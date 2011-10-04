//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Federico Barbagli
    \version   2.0.0 $Rev: 258 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDriverServotogoH
#define CDriverServotogoH
//---------------------------------------------------------------------------
#include "devices/CGenericDevice.h"
//---------------------------------------------------------------------------
#if defined(_ENABLE_SERVOTOGO_DEVICE_SUPPORT)
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDriverServotogo.h

    \brief
    <b> Devices </b> \n 
    Servo2Go IO Board.
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DLPORTIO
//---------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#define DLPORT_API _stdcall

unsigned char DLPORT_API
DlPortReadPortUchar(
    IN unsigned long Port
    );

unsigned short DLPORT_API
DlPortReadPortUshort(
    IN unsigned long Port
    );

unsigned long DLPORT_API
DlPortReadPortUlong(
    IN unsigned long Port
    );

void DLPORT_API
DlPortReadPortBufferUchar(
    IN unsigned long Port,
    IN unsigned char* Buffer,
    IN unsigned long  Count
    );

void DLPORT_API
DlPortReadPortBufferUshort(
    IN unsigned long Port,
    IN unsigned short* Buffer,
    IN unsigned long Count
    );

void DLPORT_API
DlPortReadPortBufferUlong(
    IN unsigned long Port,
    IN unsigned long* Buffer,
    IN unsigned long Count
    );

void DLPORT_API
DlPortWritePortUchar(
    IN unsigned long Port,
    IN unsigned char Value
    );

void DLPORT_API
DlPortWritePortUshort(
    IN unsigned long Port,
    IN unsigned short Value
    );

void DLPORT_API
DlPortWritePortUlong(
    IN unsigned long Port,
    IN unsigned long Value
    );

void DLPORT_API
DlPortWritePortBufferUchar(
    IN unsigned long Port,
    IN unsigned char* Buffer,
    IN unsigned long  Count
    );

void DLPORT_API
DlPortWritePortBufferUshort(
    IN unsigned long Port,
    IN unsigned short* Buffer,
    IN unsigned long Count
    );

void DLPORT_API
DlPortWritePortBufferUlong(
    IN unsigned long Port,
    IN unsigned long* Buffer,
    IN unsigned long Count
    );

#ifdef __cplusplus
}
#endif

//---------------------------------------------------------------------------

// Internal information to ServoToGo
typedef union
{
    unsigned long                       all;
    struct {unsigned char  A, B, C, D;} port;
} IO32;

// Internal information to ServoToGo
typedef union
{
    long           Long;
    unsigned char  Byte[4];
} LONGBYTE;

// Internal information to ServoToGo
typedef union
{
    unsigned short                          Word;
    struct   {unsigned char    high, low;}  Byte;
} WORDBYTE;

//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \class      cDriverServotogo
    \ingroup    devices  

    \brief      
    cDriverServotogo offers an interface to the Servo2Go boards.
*/
//===========================================================================
class cDriverServotogo : public cGenericDevice
{
  public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cDriverServotogo.
    cDriverServotogo();

    //! Destructor of cDriverServotogo.
    ~cDriverServotogo();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Open connection to Sensoray626 board.
    virtual int open();

    //! Close connection to Sensoray626 board.
    virtual int close();

    //! Initialize Sensoray626 board.
    virtual int initialize(const bool a_resetEncoders=false);

    /*!
    Send a command to the Servo2Go board. Possible commands are: \n
    \b CHAI_CMD_GET_DEVICE_STATE: returns an int (1 board is ready, 0 board is NOT ready). \n
    \b CHAI_CMD_GET_ENCODER_0: reads encoder 0, returns counts value in a long. \n
    \b CHAI_CMD_GET_ENCODER_1: reads encoder 1, returns counts value in a long. \n
    \b CHAI_CMD_GET_ENCODER_2: reads encoder 2, returns counts value in a long. \n
    \b CHAI_CMD_GET_ENCODER_3: reads encoder 3, returns counts value in a long. \n
    \b CHAI_CMD_GET_ENCODER_4: reads encoder 4, returns counts value in a long, and a value of 0 if the encoder does not exist. \n
    \b CHAI_CMD_GET_ENCODER_5: reads encoder 5, returns counts value in a long, and a value of 0 if the encoder does not exist. \n
    \b CHAI_CMD_GET_ENCODER_6: reads encoder 6, returns counts value in a long, and a value of 0 if the encoder does not exist. \n
    \b CHAI_CMD_GET_ENCODER_7: reads encoder 7, returns counts value in a long, and a value of 0 if the encoder does not exist. \n

    \b CHAI_CMD_SET_DAC_0: writes a voltage to DAC 0 a value between +10 and -10 volts, which is a double. \n
    \b CHAI_CMD_SET_DAC_1: writes a voltage to DAC 1 a value between +10 and -10 volts, which is a double. \n
    \b CHAI_CMD_SET_DAC_2: writes a voltage to DAC 2 a value between +10 and -10 volts, which is a double. \n
    \b CHAI_CMD_SET_DAC_3: writes a voltage to DAC 3 a value between +10 and -10 volts, which is a double. \n
    \b CHAI_CMD_SET_DAC_4: writes a voltage to DAC 4 a value between +10 and -10 volts, which is a double. If this axis is not supported no action is taken. \n
    \b CHAI_CMD_SET_DAC_5: writes a voltage to DAC 5 a value between +10 and -10 volts, which is a double. If this axis is not supported no action is taken. \n
    \b CHAI_CMD_SET_DAC_6: writes a voltage to DAC 6 a value between +10 and -10 volts, which is a double. If this axis is not supported no action is taken. \n
    \b CHAI_CMD_SET_DAC_7: writes a voltage to DAC 7 a value between +10 and -10 volts, which is a double. If this axis is not supported no action is taken. \n
    */
    virtual int command(int a_command, void* a_data);

  private:

    //-----------------------------------------------------------------------
    //! MEMBERS:
    //-----------------------------------------------------------------------
    
      //! Initial values of the encoder to reset them.
    long m_homeposition[20];

    //! Board base address.
    unsigned short m_wBaseAddress;

    //! Interrupt request used by board.
    unsigned short m_wIrq;

    //! Board model: defined by BrdtstOK.
    unsigned short m_wModel;

    //! Tells if the board is present. set by stg_init()
    unsigned short m_wNoBoardFlag;

    //! Number of encoders used. set by stgInit() through encoderInit().
    unsigned short m_wAxesInSys;

    //! Directions for DIO. Set by DIODirections(). (don't care now).
    unsigned short m_wSaveDirs;

    //! Set by stg_init()
    unsigned char  m_byIndexPollAxis;

    //! Polarity of signals.
    unsigned char  m_byIndexPulsePolarity;


    //-----------------------------------------------------------------------
    // METHODS
    //-----------------------------------------------------------------------

    //! Set value to dac.
    void setDac(int a_num, double a_volts);

    //! Read encoder values. Returns the number of values read.
    int getEncoder(int a_num, long *a_value);

    //! Checks if the board is present.
    unsigned short brdtstOK(unsigned short a_baseAddress);

    //! Initializes the board. This should be called by the constructor.
    void stg_Init(unsigned short a_wAdd);

    //! Initializes the encoders.
    void encoderInit();

    //! Latches the encoders.
    void encoderLatch();

    //! Automatically finds the base address of the board.
    unsigned short findBaseAddress();

    //! Latches and reads all the encoders of the board.
    void encReadAll(LONGBYTE * a_lbEnc);

    //! Write to DAC on the board
    void rawDAC(unsigned short a_nAxis, long a_lCounts);

    //! Returns the base address for the board.
    int getBaseAddress();
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif //_ENABLE_SERVOTOGO_DEVICE_SUPPORT
//---------------------------------------------------------------------------


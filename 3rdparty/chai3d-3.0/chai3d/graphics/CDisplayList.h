//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 733 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDisplayListH
#define CDisplayListH
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDisplayList.h
    
    \brief  
    <b> Graphics </b> \n 
    OpenGL Display Lists.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cDisplayList
    \ingroup    graphics

    \brief      
    cDisplayList provides a simple structure to handle the creation of an
    OpenGL dislay list.
*/
//===========================================================================
class cDisplayList
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
 
    public:

    //! Constructor of cDisplayList.
    cDisplayList();
   
    //! Destructor of cDisplayList.
    ~cDisplayList();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    public:

    //! Invalidate current display list. Request for update.
    void invalidate();

    //! Render display list.
    bool render(const bool a_useDisplayList = true);

    //! Begin creating display list.
    bool begin(const bool a_useDisplayList = true);

    //! Finalize the creatio of display list.
    void end(const bool a_executeDisplayList = true);

    //! Get OpenGL display list number.
    unsigned int getDisplayListGL() { return (m_displayList); }


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    private:
    
    //! OpenGL display list.
    unsigned int m_displayList;

    //! If \b true, then currently creating new display list.
    bool m_flagCreatingDisplayList;
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

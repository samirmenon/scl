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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "graphics/CDisplayList.h"
//---------------------------------------------------------------------------
#include "graphics/GLee.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
   Constructor of cDisplayList. The display list is set to zero as it has
   not yert been create. We also initialize the flag which tells us if a 
   display list is currently being created.
   
   \fn          cDisplayList::cDisplayList()
*/
//===========================================================================
cDisplayList::cDisplayList()
{
    m_displayList = 0;
    m_flagCreatingDisplayList = false;
}


//===========================================================================
/*!
   Destructor of cDisplayList.

   \fn          cDisplayList::~cDisplayList()
*/
//===========================================================================
cDisplayList::~cDisplayList()
{
    // delete display list
    invalidate();
}


//===========================================================================
/*!
   Invalidate current display list. We inform the display list that it will
   need to be update the next time the object is rendered. We free any 
   graphic card memory which contains the current display list.

   \fn          void cDisplayList::update()
*/
//===========================================================================
void cDisplayList::invalidate()
{
    // delete any allocated display lists
    if (m_displayList != 0)
    {
        glDeleteLists(m_displayList, 1);
    }

    // display list is invalid
    m_displayList = 0;
    m_flagCreatingDisplayList = false; 
}


//===========================================================================
/*!
   Render display list. We pass a boolean from the object which will
   check if the display list should be used. If the display list is valid
   and used for rendering the object, the method returns \b true, otherwise
   \b false.

   \fn          bool cDisplayList::render()

      \param       a_useDisplayList  If \b true, then display list is rendered.

   \return      Return \b true is display list was rendered, otherwise \b false
                if display list was invalid. 
*/
//===========================================================================
bool cDisplayList::render(const bool a_useDisplayList)
{
    if ((a_useDisplayList) && (m_displayList != 0))
    {
        glCallList(m_displayList);
        return (true);
    }
    else
    {
        return (false);
    }
}


//===========================================================================
/*!
   Begin creating a display list. 

   \fn          bool cDisplayList::begin()

   \param       a_useDisplayList  If \b true, then display list is created.

   \return      Return \b true if the OpenGL display list has been allocated
                successfully.
*/
//===========================================================================
bool cDisplayList::begin(const bool a_useDisplayList)
{
    if (a_useDisplayList)
    {
        // clear any current display list
        invalidate();

        // create an OpenGL display list
        m_displayList = glGenLists(1);

        // verify result
        if (m_displayList == 0) 
        {
            // operation failed
            return (false);
        }
        else
        {
            // On some machines, GL_COMPILE_AND_EXECUTE totally blows for some reason,
            // so even though it's more complex on the first rendering pass, we use
            // GL_COMPILE (and _repeat_ the first rendering pass)
            glNewList(m_displayList, GL_COMPILE);

            // we are now creating a display list
            m_flagCreatingDisplayList = true;

            // success
            return (true);
        }
    }
    else
    {
        return (false);
    }

}


//===========================================================================
/*!
   Finalize and compile the display list. Optionally render the display list
   as nothing will have yet been rendered at the screen during display
   list compilation which began after calling method begin().
   
   \fn          void cDisplayList::end(bool a_executeDisplayList)
   \param       a_executeDisplayList  If \b true, render display list.
*/
//===========================================================================
void cDisplayList::end(const bool a_executeDisplayList)
{
    if (m_flagCreatingDisplayList)
    {
        // finalize list
        glEndList();

        // display list has been finalized
        m_flagCreatingDisplayList = false;

        // execute display list if requested
        if (a_executeDisplayList)
        {
            render();
        }
    }
}

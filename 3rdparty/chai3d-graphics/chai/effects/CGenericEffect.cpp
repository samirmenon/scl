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
    \author    Francois Conti
    \version   2.0.0 $Rev: 244 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "effects/CGenericEffect.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cGenericEffect.

    \fn  cGenericEffect::cGenericEffect(cGenericObject* a_parent);
    \param  a_parent Parent object.
*/
//===========================================================================
cGenericEffect::cGenericEffect(cGenericObject* a_parent)
{
    // set parent object
    m_parent = a_parent;

    // no force has been computed yet
    m_lastComputedForce.zero();

    // effect is enabled by default
    m_enabled = true;

    // initialize force effect
    initialize();
}




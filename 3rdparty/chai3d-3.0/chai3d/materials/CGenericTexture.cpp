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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 322 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CGenericTexture.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Texture Filter Parameters Explained:
//---------------------------------------------------------------------------
/*
    The value of parameter number three depends on whether you are setting a 
    GL_TEXTURE_MIN_FILTER variable or a GL_TEXTURE_MAG_FILTER variable. 
    Here are the possible combinations:

    When the second parameter is GL_TEXTURE_MIN_FILTER, parameter three can be:
        GL_NEAREST_MIPMAP_NEAREST
        GL_LINEAR_MIPMAP_NEAREST
        GL_NEAREST_MIPMAP_LINEAR
        GL_LINEAR_MIPMAP_LINEAR
        GL_NEAREST
        GL_LINEAR
    When the second parameter is GL_TEXTURE_MAG_FILTER, parameter three can be:
        GL_LINEAR
        GL_NEAREST

The filter value set for GL_TEXTURE_MIN_FILTER is used whenever a surface is 
rendered with smaller dimensions than its corresponding texture bitmap 
(far away objects). Whereas the filter value for GL_TEXTURE_MAG_FILTER is used in 
the exact opposite case – a surface is bigger than the texture being applied (near objects).

There are more options for the min filter because it can potentially have mipmapping. 
However, it wouldn’t make sense to apply mipmapping to the mag filter since close-up 
objects don’t need it in the first place. Here’s a list of all the possible combinations 
and how they impact what is rendered (first constant in the left-most column is the near 
object filter [mag]; second constant is the far object filter [min]):


Filter Combination                          Bilinear Filtering    Bilinear Filtering    Mipmapping
(MAG_FILTER/MIN_FILTER) 	                (Near) 	              (Far) 	
GL_NEAREST / GL_NEAREST_MIPMAP_NEAREST 	    Off 	              Off 	                Standard
GL_NEAREST / GL_LINEAR_MIPMAP_NEAREST 	    Off 	              On 	                Standard
GL_NEAREST / GL_NEAREST_MIPMAP_LINEAR 	    Off 	              Off 	                Use trilinear filtering
GL_NEAREST / GL_LINEAR_MIPMAP_LINEAR 	    Off 	              On 	                Use trilinear filtering
GL_NEAREST / GL_NEAREST 	                Off 	              Off 	                None
GL_NEAREST / GL_LINEAR 	                    Off 	              On 	                None
GL_LINEAR / GL_NEAREST_MIPMAP_NEAREST 	    On 	                  Off 	                Standard
GL_LINEAR / GL_LINEAR_MIPMAP_NEAREST 	    On 	                  On 	                Standard
GL_LINEAR / GL_NEAREST_MIPMAP_LINEAR 	    On 	                  Off 	                Use trilinear filtering
GL_LINEAR / GL_LINEAR_MIPMAP_LINEAR 	    On 	                  On 	                Use trilinear filtering
GL_LINEAR / GL_NEAREST 	                    On 	                  Off 	                None
GL_LINEAR / GL_LINEAR 	                    On 	                  On 	                None
*/
//---------------------------------------------------------------------------
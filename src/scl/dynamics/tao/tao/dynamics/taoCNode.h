/* Copyright (c) 2005 Arachi, Inc. and Stanford University. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _taoCNode_h
#define _taoCNode_h

#include <string>

#include <tao/matrix/TaoDeTypes.h>

class deVector3;
class deFrame;

/*!
 *	\brief		Contact node class
 *	\ingroup	taoDynamics
 *
 *	This provides a base node class for other node involving collision.
 *	\sa	taoDNode, taoNode, taoNodeRB, taoNodePS
 */
class taoCNode
{
public:
  std::string name_;

	taoCNode() {}
	virtual ~taoCNode() {}

	//!	global frame for the dynamics computation
	/*!	
	 *	\note	for taoNodeRB, this is the center of mass frame.
	 *	\note	for taoNode, this is the same frame getFrameGraphics()
	 */
	virtual deFrame* frameGlobal() = 0;
	virtual deFrame const * frameGlobal() const = 0;
	//!	global frame for graphics display
	/*!
	 *	\note	for taoNodeRB, this is the graphics origin frame without the offset.
	 *	\note	for taoNodeRB, setFrameGraphics() should be used to set this frame.
	 *	\note	for taoNode, this is the same frame as frameGlobal()
	 *	\retval	Fg is filled with the frame info for graphics sync.	
	 */
	virtual void getFrameGraphics(deFrame* Fg) = 0;


};

#endif // _taoCNode_h

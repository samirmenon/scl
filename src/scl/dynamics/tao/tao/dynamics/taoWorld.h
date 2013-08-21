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

/** Edited 2013-08-20 : Samir Menon <smenon@stanford.edu>
 */

#ifndef _taoWorld_h
#define _taoWorld_h

#include "taoTypes.h"
#include <tao/matrix/TaoDeMath.h>

class taoDNode;
class taoNode;
class taoNodeRoot;

/*!
 *	\brief container class to hold dynamics characters
 *	\ingroup taoDynamics
 *
 *	A group is a container object for dynamics characters, particle systems, 
 *	rigid bodies, and articulated bodies. Characters in different groups can 
 *	not interact, e.g., no collision between characters from two different groups. 
 *	All characters in a group share common parameters such as integration time step, gravity, etc.
 */
class taoWorld
{
public:
	taoWorld() : _id(-1), _isFixed(0), root_node_(NULL), _next(NULL) { _gravity.zero(); }
	~taoWorld();

	void setID(deInt i) { _id = i; }
	const deInt getID() const { return _id; }

	void setIsFixed(int f) { _isFixed = f; }
	deInt getIsFixed() { return _isFixed; }

	deVector3* gravity() { return &_gravity; }

	void setNext(taoWorld* g) { _next = g; }
	taoWorld* getNext() { return _next; }

	taoNodeRoot* getRootList() { return root_node_; }
	void addRoot(taoNodeRoot* r, const deInt id);
	taoNodeRoot* removeRoot(const deInt id);
	taoNodeRoot* findRoot(const deInt id);

	/*!
	 *  \remarks  this can be replaced by following 3 individual call.
	 *  \remarks  simulate(), updateTransformation()
	 *
	 *  \arg  time  control desired goal achieving time. this value
	 *          is used to compute the goal frames.
	 *          Also, this value should be greater than the last
	 *          control time, taoControl::time() and less than equal
	 *          to the current goal time set by taoControl::setGoalPosition().
	 *  \arg  dt    integration time step.  notice that this value is independent to \a time.
	 *  \arg  n   number of iteration of the loop if necessary
	 */
	void update(const deFloat time, const deFloat dt, const deInt n);
	void simulate(const deFloat dt);
	void updateTransformation();

	taoNodeRoot* unlinkFixed(taoNodeRoot* root, taoNode* node);
	taoNodeRoot* unlinkFree(taoNodeRoot* root, taoNode* node, deFloat inertia, deFloat damping);

	void sync(taoNodeRoot* root, deFloat time);

private:
	deInt _id;
	deInt _isFixed;
	deVector3 _gravity;
	taoNodeRoot* root_node_;

	taoWorld* _next;
};

#endif // _taoWorld_h

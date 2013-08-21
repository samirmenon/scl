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

/** Edited 2013-08-21 : Samir Menon <smenon@stanford.edu>
 */

#ifndef _taoDNode_h
#define _taoDNode_h

#include "taoTypes.h"

class taoArticulatedBodyLink;
class taoJoint;
class deVector3;
class deVector6;
class deFrame;
class deMatrix3;

/*!
 *	\brief abstract node class for articulated body
 *	\ingroup taoDynamics
 *
 *  A dynamics node = {Articulated Body Link + Set of associated Joints}.
 *
 *	This class should be used as a base class and implemented accordingly.	
 */
class taoDNode
{
public:
	taoDNode() : _abNode(NULL), _propagate(1),
	/** Copied from CNode */
	_id(-1), _isFixed(0), _cor(0), _cofg(0), _cofv(0), _cofs(0), _cofd(0) {}

	virtual ~taoDNode() {}

	virtual void sync(deFrame* local) = 0;
	//!	indicates if this node is root
	/*!	\retval	1	this node is root
	 *	\retval	0	this node in not root
	 */
	virtual deInt isRoot() { return (getDParent() == NULL); }
	//!	indicates if parent of this node is root
	/*!	\retval	1	parent this node is root
	 *	\retval	0	parent this node in not root
	 */
	virtual deInt isParentRoot() { return (isRoot() || getDParent()->isRoot()); }

	virtual taoJoint* getJointList() = 0;
	virtual taoJoint const * getJointList() const = 0;

	//!	gets _abNode
	virtual taoArticulatedBodyLink* getABNode() { return _abNode; }
	//!	sets _abNode
	virtual void setABNode(taoArticulatedBodyLink* node) { _abNode = node; }

	//!	indicates if dynamics of node should be included for control
	/*!	\retval	1	to propagate dynamics of node for control 
	 *	\retval	0	not to propagate dynamics of node for control
	 */
	virtual deInt getPropagate() { return _propagate; }
	virtual void setPropagate(deInt p) { _propagate = p; } // YYY

	virtual deVector6* velocity() = 0;
	virtual deVector6* acceleration() = 0;

	//! \return	home frame
	virtual deFrame* frameHome() = 0;
	//! \return	local frame
	virtual deFrame* frameLocal() = 0;
	//! \return	global frame
	//virtual deFrame* frameGlobal() = 0;
	//! \return	mass
	virtual deFloat* mass() = 0;
	//! \return	center of gravity in local frame
	virtual deVector3* center() = 0;
	virtual deVector3 const * center() const = 0;
	//!	\return	inertia tensor at the origin of local frame
	virtual deMatrix3* inertia() = 0;
	//!	\retval	fext	accumulate all external forces
	virtual deVector6* force() = 0;
	virtual void zeroForce() = 0;
	virtual void addForce(const deVector6* f) = 0;

	virtual void updateFrame() = 0;
	virtual void integrate(deFloat dt) = 0;

	//!	\return	parent node
	virtual taoDNode* getDParent() = 0;
	virtual taoDNode const * getDParent() const = 0;
	virtual void setDChild(taoDNode* n) = 0;
	//!	\return	first child node
	virtual taoDNode* getDChild() = 0;
	virtual taoDNode const* getDChild() const = 0;
	//!	\return	next sibling node
	virtual taoDNode* getDSibling() = 0;
	virtual taoDNode const* getDSibling() const = 0;

private:
	taoArticulatedBodyLink* _abNode;
	deInt _propagate;

	/* *********************************************
	 *           Moved here from taoDNode
	 * ********************************************* */
public:
	//! global frame for the dynamics computation
	/*!
	 *  \note for taoNodeRB, this is the center of mass frame.
	 *  \note for taoNode, this is the same frame getFrameGraphics()
	 */
	virtual deFrame* frameGlobal() = 0;
	virtual deFrame const * frameGlobal() const = 0;
	//! global frame for graphics display
	/*!
	 *  \note for taoNodeRB, this is the graphics origin frame without the offset.
	 *  \note for taoNodeRB, setFrameGraphics() should be used to set this frame.
	 *  \note for taoNode, this is the same frame as frameGlobal()
	 *  \retval Fg is filled with the frame info for graphics sync.
	 */
	virtual void getFrameGraphics(deFrame* Fg) = 0;

	virtual void setID(const deInt id) { _id = id; }
	virtual deInt getID() const { return _id; }

	virtual void setIsFixed(const deInt f) { _isFixed = f; }
	virtual deInt getIsFixed() { return _isFixed; }

	virtual deFloat getCOR() { return _cor; }
	virtual deFloat getCOF_grip() { return _cofg; }
	virtual deFloat getCOF_viscous() { return _cofv; }
	virtual deFloat getCOF_static() { return _cofs; }
	virtual deFloat getCOF_dynamic() { return _cofd; }

	//! coefficient of restitution
	virtual void setCOR(deFloat c) { _cor = c; }
	/*!
	 *  \name Coefficients of friction
	 *  where @c v is velocity, @c a is acceleration, @c y is impulse, and @c f is force at the contact point.
	 *  @note: @c (*)_n and @c (*)_t indicate the normal and tangential components of @c (*) respectively.
	 */
	//! @c cofg : coefficient of grip friction when <tt> v != 0 </tt> and <tt> y != 0 </tt> :  <tt> f_g = - sign(v_t) * @c cofg  * y_n </tt>
	virtual void setCOF_grip(deFloat c) { _cofg = c; }
	//! @c cofv : coefficient of viscous friction when <tt> v != 0 </tt> :  <tt> f_v = - v_t * @c cofv </tt>
	virtual void setCOF_viscous(deFloat c) { _cofv = c; }
	//! @c cofs : coefficient of static friction when <tt> v == 0 </tt> and <tt> a != 0 </tt> :  <tt> f_s = - sign(a_t) * @c cofs * f_n </tt>
	virtual void setCOF_static(deFloat c) { _cofs = c; }
	//! @c cofd : coefficient of dynamic friction when <tt> v != 0 </tt> :  <tt> f_d = - sign(v_t) * @c cofd * f_n </tt>
	virtual void setCOF_dynamic(deFloat c) { _cofd = c; }

	virtual deFloat effectiveMass(const deVector3* Pie, const deVector3* Ui) { return 0; }
	virtual void linearVelocity(deVector3* Vie, const deVector3* Pie) {}
	virtual void linearAcceleration(deVector3* Aie, const deVector3* Pie) {}
	//! computes new velocity given impulse
	/*!
	 *  This method changes velocity instantaneously by applying the given impulse.
	 *  \remarks  do not use ddQ after this call
	 *  \pre  T, Q, V, dQ, Y
	 *  \post V, dQ
	 *
	 *  @arg  @c Pie - contact point expressed in local frame.
	 *  @arg  @c Yie - impulse at the contact point expressed in local frame.
	 */
	virtual void impulse(const deVector3* Pie, const deVector3* Yie) {}
	//! computes new position given pseudo impulse
	/*!
	 *  This method changes position and orientation instantaneously by applying the given pseudo impulse.
	 *
	 *  @arg  @c Pie - contact point expressed in local frame.
	 *  @arg  @c Yie - pseudo impulse at the contact point expressed in local frame.
	 */
	virtual void impulseDist(const deVector3* Pie, const deVector3* Yie) {}
	//! computes new force given force
	/*!
	 *  This method replaces the accumulated force with the given force.
	 *  \remarks  replaces the internal force with a new force
	 *  \pre  A, F
	 *  \post F
	 *
	 *  @arg  @c Pie - contact point expressed in local frame.
	 *  @arg  @c Fie - force at the contact point expressed in local frame.
	 *  \note this method \em replaces the internal force in the node.
	 */
	virtual void force(const deVector3* Pie, const deVector3* Fie) {}
	//! computes new velocity and force given pos, vel, acc
	/*!
	 *  \remarks  do not use ddQ after this call, replaces the internal force to the new force
	 *  \pre  T, Q, V, dQ, A
	 *  \post V, dQ, F
	 *
	 *  @arg  @c Pie - contact point expressed in local frame.
	 *  @arg  @c Ui - unit contact direction vector expressed in local frame (outward normal vector)
	 *  @arg  @c cor2 - coefficient of restitution of the other collision node.
	 *  @arg  @c cof2 - coefficient of grip friction of the other collision node.
	 *  \note this method is assuming a node is colliding with a non-moving node.
	 */
	virtual deInt impact1(const deVector3* Pie, const deVector3* Ui, const deFloat cor2, const deFloat cofg2);
	//! This method computes impulse/force and changes velocity/friction between two colliding nodes, \a ni and \a nj
	/*!
	 *  @arg  @c Pie - contact point expressed in frame \em i : <tt> Pie = ^iP_e - ^iP_i </tt>
	 *  @arg  @c Pje - contact point expressed in frame \em j : <tt> Pje = ^jP_e - ^jP_j </tt>
	 *  @arg  @c Ui - unit contact direction vector expressed in frame \em i : <tt> Ui = (^iP_j - ^iP_i)/|^iP_j - ^iP_i| </tt>
	 *  @arg  @c Uj - \c Ui expressed in frame \em j
	 *  \note this method should be used for two moving nodes.
	 */
	virtual deInt impact2(taoDNode* ni, const deVector3* Pie, const deVector3* Ui,
	    taoDNode* nj, const deVector3* Pje, const deVector3* Uj);
	//! This method computes impulse and changes position/orientation using @c impulseDist().
	/*!
	 *  @arg  @c Pie - contact point expressed in local frame.
	 *  @arg  @c Ui - unit contact direction vector expressed in local frame (outward normal vector).
	 *  @arg  @c pdist - penetration distance vector : <tt> pdist = P_o - P_i </tt>
	 *  @arg  @c dt - integration time step.
	 *  \note this method is assuming a node is colliding with a non-moving node.
	 */
	virtual deInt penetration1(const deVector3* Pie, const deVector3* Ui, const deVector3* pdist, const deFloat dt);

	std::string name_;

private:
	deInt _id;
	deInt _isFixed;

	deFloat _cor;
	deFloat _cofg;
	deFloat _cofv;
	deFloat _cofs;
	deFloat _cofd;

	deInt _Impact1(const deVector3* Pie, const deVector3* Ui, const deVector3* V, const deFloat m, const deFloat vp, const deFloat vm, const deFloat cofg2);
	void _Friction(const deVector3* Pie, const deVector3* Ui, const deVector3* Ut, const deFloat vtmag, const deFloat m);
};

#endif // _taoDNode_h

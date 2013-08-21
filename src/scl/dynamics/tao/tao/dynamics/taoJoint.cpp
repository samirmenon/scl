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

#include <tao/dynamics/taoJoint.h>
#include "taoABJoint.h"

taoJoint::~taoJoint() 
{ 
	delete _var;
	//delete _abJoint;  // done in ~taoABNode()
}

taoJointSpherical::taoJointSpherical() 
{ 
	setABJoint(new taoABJointSpherical(this)); 
	setType(scl::JOINT_TYPE_SPHERICAL);
}

void taoJointSpherical::reset()
{
	getVarSpherical()->q_quat_.identity();
	getVarSpherical()->dq_.zero();
	getVarSpherical()->ddq_.zero();
	getVarSpherical()->force_gc_.zero();
	getVarSpherical()->dq_rotated_.zero();
}

void taoJointSpherical::addQdelta()
{
	deVector3 dq0;
	deQuaternion dq;

	//NOTE TODO: The code after this can be simplified this way.
  //static_cast<taoVarSpherical*> (
  //   static_cast<taoDVar*>(_var)
  //   );
  //taoVarSpherical* t = static_cast<taoVarSpherical*>(_var);
  //dq0.multiply(t->_Q, t->_ddQ);
  //dq.velocity(t->_Q, dq0);
  //t->_Q += dq;    // YYY: consistentSign(); ?
  //t->_Q.normalize();

	/** Another example of obfuscated code.
	 * The one above code probably does the same thing. */
	dq0.multiply(getVarSpherical()->q_quat_, getVarSpherical()->ddq_);
	dq.velocity(getVarSpherical()->q_quat_, dq0);
	getVarSpherical()->q_quat_ += dq;		// YYY: consistentSign(); ?
	getVarSpherical()->q_quat_.normalize();
}

void taoJointSpherical::addDQdelta()
{
	deVector3 ddq;
	ddq.multiply(getVarSpherical()->q_quat_, getVarSpherical()->ddq_);
	getVarSpherical()->dq_rotated_ += ddq;
	getVarSpherical()->dq_.inversedMultiply(getVarSpherical()->q_quat_, getVarSpherical()->dq_rotated_);

	if (getDQclamp()) 
		clampDQ();
}

void taoJointSpherical::clampDQ()
{
	deInt changed = 0;
	for (deInt i = 0; i < 3; i++)
		if (getVarSpherical()->dq_[i] > getDQmax())
		{
			getVarSpherical()->dq_[i] = getDQmax();
			changed = 1;
		}
		else if (getVarSpherical()->dq_[i] < -getDQmax())
		{
			getVarSpherical()->dq_[i] = -getDQmax();
			changed = 1;
		}
	if (changed)
		getVarSpherical()->dq_rotated_.multiply(getVarSpherical()->q_quat_, getVarSpherical()->dq_);
}

void taoJointSpherical::integrate(const deFloat dt)
{
	deQuaternion dq;
	dq.velocity(getVarSpherical()->q_quat_, getVarSpherical()->dq_rotated_);
	dq *= dt;
	getVarSpherical()->q_quat_ += dq;		// YYY: consistentSign(); ?
	getVarSpherical()->q_quat_.normalize();

	deVector3 ddq;
	ddq.multiply(getVarSpherical()->q_quat_, getVarSpherical()->ddq_);
	deVector3 tmp;	
	tmp.multiply(ddq, dt);
	getVarSpherical()->dq_rotated_ += tmp;

	getVarSpherical()->dq_.inversedMultiply(getVarSpherical()->q_quat_, getVarSpherical()->dq_rotated_);

	if (getDQclamp()) 
		clampDQ();
}

deMatrix3* taoJointSpherical::getJg()
{
	return ((taoABJointSpherical*)getABJoint())->Jg();
}

void taoJointSpherical::getJgColumns(deVector6 * Jg_columns) const
{
  deMatrix3 const * Jg_p(((taoABJointSpherical*)getABJoint())->Jg());
  deMatrix3 const * Jg_w(Jg_p + 1);
  for (size_t icol(0); icol < 3; ++icol) {
    deVector3 & col_p(Jg_columns[icol][0]);
    deVector3 & col_w(Jg_columns[icol][1]);
    for (size_t isubrow(0); isubrow < 3; ++isubrow) {
      col_p[isubrow] = Jg_p->elementAt(isubrow, icol);
      col_w[isubrow] = Jg_w->elementAt(isubrow, icol);
    }
  }
}

deVector6& taoJointDOF1::getJg() const
{
	return ((taoABJointDOF1*)getABJoint())->Jg();
}

void taoJointDOF1::getJgColumns(deVector6 * Jg_columns) const
{
  deVector6 const & Jg(((taoABJointDOF1*)getABJoint())->Jg());
  *Jg_columns = Jg;
}

deVector6& taoJointDOF1::getS()
{
	return ((taoABJointDOF1*)getABJoint())->S();
}

void taoJointDOF1::integrate(const deFloat dt)
{
	getVarDOF1()->q_ += getVarDOF1()->dq_ * dt;
	getVarDOF1()->dq_ += getVarDOF1()->ddq_ * dt;

	if (getDQclamp()) 
		clampDQ();
}

void taoJointDOF1::clampDQ()
{
	if (getVarDOF1()->dq_ > getDQmax())
		getVarDOF1()->dq_ = getDQmax();
	else if (getVarDOF1()->dq_ < -getDQmax())
		getVarDOF1()->dq_ = -getDQmax();
}

taoJointPrismatic::taoJointPrismatic(taoAxis axis) : taoJointDOF1(axis) 
{ 
	setABJoint(new taoABJointPrismatic(axis, this));
	switch (axis)
	{
	  case TAO_AXIS_X:
	    setType(scl::JOINT_TYPE_PRISMATIC_X);
	    break;

    case TAO_AXIS_Y:
      setType(scl::JOINT_TYPE_PRISMATIC_Y);
      break;

    case TAO_AXIS_Z:
      setType(scl::JOINT_TYPE_PRISMATIC_Z);
      break;

    default:
      setType(scl::JOINT_TYPE_NOTASSIGNED);
      break;
	}
}

taoJointRevolute::taoJointRevolute(taoAxis axis) : taoJointDOF1(axis)
{ 
	setABJoint(new taoABJointRevolute(axis, this));
  switch (axis)
  {
    case TAO_AXIS_X:
      setType(scl::JOINT_TYPE_REVOLUTE_X);
      break;

    case TAO_AXIS_Y:
      setType(scl::JOINT_TYPE_REVOLUTE_Y);
      break;

    case TAO_AXIS_Z:
      setType(scl::JOINT_TYPE_REVOLUTE_Z);
      break;

    default:
      setType(scl::JOINT_TYPE_NOTASSIGNED);
      break;
  }
}

taoJointUser::taoJointUser() : taoJointDOF1(TAO_AXIS_USER)
{ 
	setABJoint(new taoABJointDOF1(this));
	setType(scl::JOINT_TYPE_NOTASSIGNED);
}

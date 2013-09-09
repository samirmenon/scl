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

#ifndef _deFrame_inl
#define _deFrame_inl
	
DE_MATH_API void deFrame::identity() { ori_quat_.identity(); translation_.zero(); }
DE_MATH_API void deFrame::operator=(const deFrame & f) { ori_quat_ = f.ori_quat_; translation_ = f.translation_; } 
//! this = f1 * f2 = [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
DE_MATH_API void deFrame::multiply(const deFrame& f1, const deFrame& f2) {
	ori_quat_.multiply(f1.rotation(), f2.rotation());
	translation_.multiply(f1.rotation(), f2.translation());
	translation_ += f1.translation();
}
//! this = f1^-1 * f2 
//       = ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*p2 - (~r1*p1)]
//       = [~r1*r2, ~r1*(p2-p1)]
DE_MATH_API void deFrame::inversedMultiply(const deFrame& f1, const deFrame& f2) {
	deVector3 p;
	p.subtract(f2.translation(), f1.translation());
	translation_.inversedMultiply(f1.rotation(), p);
	ori_quat_.inversedMultiply(f1.rotation(), f2.rotation());
}
//! this = f1 * f2^-1 
//       = [r1,p1]~[r2,p2] = [r1,p1][~r2, -(~r2*p2)] = [r1*~r2, -r1*(~r2*p2) + p1]
//       = [r1*~r2, -r1*~r2*p2 + p1]
DE_MATH_API void deFrame::multiplyInversed(const deFrame& f1, const deFrame& f2) {
	ori_quat_.multiplyInversed(f1.rotation(), f2.rotation());
	translation_.multiply(ori_quat_, f2.translation());
	translation_.subtract(f1.translation(), translation_);
}
//! this = f^-1 =  ~[r,p] = [~r, -(~r*p)]
DE_MATH_API void deFrame::inverse(const deFrame& f) {
	ori_quat_.inverse(f.rotation());
	translation_.multiply(ori_quat_, f.translation());
	translation_.negate(translation_);
}
DE_MATH_API void deFrame::set(const deTransform& t) { ori_quat_.set(t.rotation()); translation_ = t.translation(); }
DE_MATH_API void deFrame::set(const deQuaternion& q, const deVector3& v) { ori_quat_ = q; translation_ = v; }

#endif // _deFrame_inl


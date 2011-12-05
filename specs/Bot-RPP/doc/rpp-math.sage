# This document implements the math for the RPP
# test robot. The math includes kinematic transformations,
# dynamics matrices and the Jacobian.
# Please compile the accompanying rpp.tex file to get
# more details.

# Author: Samir Menon, smenon@stanford.edu
# Date: Nov, 2011

# All units are SI (kg, m etc..)
# The ground is the -1'th link (doesn't move)
# The robot's links are numbered starting at zero

print "*****************************************"
print "************** RPP BOT ******************"
print "*****************************************"

# The RPP has three links, a revolute, a prismatic
# and another prismatic. They are numbered 0,1,2.
# Also, some constants are:
# m0 = 1 kg
# m1 = 0.5 kg
# m2 = 0.5 kg

# Link 0: Revolute, rotates about z
# The generalized coordinate (variable)
q0 = var('q0')
# The distance of the frame from its parent
# IMPORTANT:
# Note that by default, we use the scalar along 
# the axis of rotation/translation of the frame.
l0 = vector(SR, 3, var('l0x,l0y,l0z'))
# The mass of the link
m0 = var('m0')
# The inertia of the link (A symmetric matrix)
I0 = matrix(SR,3,3,var('I0xx,I0xy,I0xz,I0xy,I0yy,I0yz,I0xz,I0yz,I0zz'))
# The center of mass of the link wrt the frame
cm0 = vector(SR, 3, var('cm0x,cm0y,cm0z'))

# Link 1: Prismatic, translates along y (see link0's comments)
q1 = var('q1')
l1 = vector(SR, 3, var('l1x,l1y,l1z'))
m1 = var('m1')
I1 = matrix(SR,3,3,var('I1xx,I1xy,I1xz,I1xy,I1yy,I1yz,I1xz,I1yz,I1zz'))
cm1 = vector(SR, 3, var('cm1x,cm1y,cm1z'))

# Link 2: Prismatic, translates along z (see link0's comments)
q2 = var('q2')
l2 = vector(SR, 3, var('l2x,l2y,l2z'))
m2 = var('m2')
I2 = matrix(SR,3,3,var('I2xx,I2xy,I2xz,I2xy,I2yy,I2yz,I2xz,I2yz,I2zz'))
cm2 = vector(SR, 3, var('cm2x,cm2y,cm2z'))

# The kinematic transformation matrices 

# To go from frame 0 to frame 1 requires a rotation about z.
# Note, the rotation q0 is about frame -1 (ground), but 
# vec_frame-1 = T-10*vec_frame0
TGr0 = Matrix([[cos(q0), -sin(q0), 0, 0],[sin(q0), cos(q0), 0, 0],[0, 0, 1, l0z],[0, 0, 0, 1]])
print "Transformation from frame 0 to ground"
print TGr0

# Frame 1 to 2 requires a translation about y
T01 = Matrix([[1, 0, 0, 0],[0, 1, 0, l1y+q1],[0, 0, 1, 0],[0, 0, 0, 1]])
print "Transformation from frame 1 to 0"
print T01

# Frame 1 to 2 requires a translation about z
T12 = Matrix([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, l2z+q2],[0, 0, 0, 1]])
print "Transformation from frame 2(end-effector) to 1"
print T12

# The overall transformation from the base to the 
TGr2 = TGr0 * T01 * T12
print "Transformation from end-effector to ground"
print TGr2

# This prints the position of the end-effector frame in
# ground frame coordinates
ee = var('ee')
ee = TGr2 * vector([0, 0, 0, 1])
print "End-effector position in base frame:"
print "x = " + str(ee[0])
print "y = " + str(ee[1])
print "z = " + str(ee[2])

# Differentiating the position in the ground frame with 
# respect to the generalized coordinates gives the Jacobian,
# which relates changes in the gen coords to changes in the
# end-effector frame's velocity.
J = ee.function(q0,q1,q2).diff()
#Weird sage seems to append one extra col (from the affine part)
J = J.submatrix(0,0,3,3)
print "Jacobian:"
print J

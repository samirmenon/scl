Dynamics Benchmarks :

1. Integrator accuracy : Test energy gained/lost 
(<1% energy error required at the end of all tests -- adjust integrator appropriately)
Timescales (sec) : 1, 10, 100, 1000, 10000, 100000

2. Integrator performance : Test time taken for all timescales above
sup(Energy error %) : <0.001, 0.01, 0.1, 1, 10

3. Articulated body test : Test all above for Pendulum models:
Models : specs/Pendulums 
* Eg. Pend24x4.xml is a quadruped with 24 dof in each arm.


Makefiles:

This is an example app. So there are two make types (supported by us)
(a) Makefile
#Debug (lots of debugging symbols)
$ make
#Release (faster)
$ make release

(b) CMake
#Debug
$ sh make_dbg.sh
#Release
$ sh make_rel.sh

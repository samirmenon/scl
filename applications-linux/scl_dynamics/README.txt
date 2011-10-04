Dynamics Benchmarks :

1. Integrator accuracy : Test energy gained/lost 
(<1% energy error required at the end of all tests -- adjust integrator appropriately)
Timescales (sec) : 1, 10, 100, 1000, 10000, 100000

2. Integrator performance : Test time taken for all timescales above
sup(Energy error %) : <0.001, 0.01, 0.1, 1, 10

3. Articulated body test : Test all above for Puma models:
Models : Puma.xml PumaX2.xml PumaX4.xml PumaX8.xml PumaX16.xml
         PumaP2.xml PumaP4.xml PumaP8.xml PumaP16.xml
         PumaX2P2.xml PumaX4P4.xml PumaX8P8.xml PumaX16P16.xml
* Where X = Pumas attached serially (extended)
*       P = Pumas attached in parallel (at the 1st non-fixed joint)
* Eg. PumaX8P8.xml is an octopus with 6*8 dof in each arm.

4. Controller test : Control Puma end-effectors (<1% error allowed, all 
                     timescales) to move along an arbitrary 3d line in 
                     a sinusoid. Amplitude is the length of the longest
                     puma link.

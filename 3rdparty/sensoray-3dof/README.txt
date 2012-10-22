2012-10-21 : Sensoray library for controlling analog out and 
             digital in for the BFR 1.
             
This is Samir's mod of the Sensoray driver. 

How to use:
1. Attach lan connection to the BFR box.
2. Set your own IP to 10.10.10.255
3. Do the following:
  $ cd applications-linux/lib
  $ sh make_dbg.sh
  $ sh make_rel.sh
  $ cd ../demo
  $ sh make_rel.sh
  $ ./sensoray-demo
4. Should connect to the sensoray modules

Why modify the defaults?
1. No source is changed. Merely the directory structure.
2. Enable #include <sensoray/...h>
3. Enable building shared and static libs
3. Keep lib directory placement consistent with rest of system.
   (Simplifies cmake build code in apps).
   Lib dir : lib/debug or lib/release
4. Provide a simple abstract header that simplifes a 3dof control 
   system.
   
NOTE : The current Sensoray driver doesn't work when compiled 
in 64 bit mode. On x86-64 machines, you have to compile it as a 
32 bit lib.


=== Historical ====

When Sensoray gave us the code, they included a short script to 
compile the lib:

"sudo apt-get install libc6-dev-i386 "
"sudo apt-get install gcc-multilib g++-multilib ia32-libs" (Samir's extras)
"rm *.o"
"./makelib32"
"./makedemo32"


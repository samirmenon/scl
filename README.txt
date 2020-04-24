scl: Standard Control Library

A control and interactive simulation library. Supports torque control on a variety of robots including
Unimate PUMA 500, Barrett WAM, Kinova Jaco, Neurobot {v1, v2, v3}, HFI {v1, v2, v2.5, v3, v5}, Kuka LWR, 
and Kuka LBR IIWA. (Low-level robot drivers must be obtained separately).

Wiki: https://bitbucket.org/samirmenon/scl-manips-v2/wiki

==================================================================================================
== Getting started ==

=== 1.Install additional dependencies. ===
Open a terminal (ctrl+alt+t) and type:
$ sudo apt-get install build-essential cmake libusb-dev libpci-dev freeglut3-dev libncurses5-dev git-core gitk libusb-1.0-0-dev libglew-dev libqhull-dev libhiredis-dev libjsoncpp-dev libc6-dev-i386 redis-server redis-tools python-redis gcc-multilib g++-multilib

===  2. Get the code. ===
I will assume you will get this on to your documents folder. Again, on the terminal:

$ cd ~/Documents
$ git clone https://github.com/samirmenon/scl.git scl.git

=== Optional : Check out a stable version of the code ===
Rember, this step is __optional__. If a tag doesn't work for you, try using the latest branch (unstable in general but very likely to have your problem fixed).
Alternatively try an older tag or file some bugs.
$ git tag -l

Then pick whatever tag (stable point) you want. Preferably pick the most recent one.
$ git checkout -b tag-v1.00 tags/v1.00

Unless you have some experience coding, you should consider using a stable version of the code.


=== 2.b Compile 3rdparty/chai3d.git  ===
The graphics rendering uses chai3d. However, chai3d (3.2) requires a lot of RAM to compile. So we're adding it as a separate step instead of using a submodule. Note, chai was packaged as a 3rdparty earlier so if you'd like ease of compile, use an older version of SCL (chai3d 3.0; easier compile).
$ cd 3rdparty
$ git clone git@github.com:samirmenon/chai3d.git chai3d.git
$ cd chai3d.git
$ sudo apt install xorg-dev libasound2-dev
$ sh make_everything.sh
$ cd ../../

NOTE : Due to updates in the package libs installed on Ubuntu. Chai might throw compile errors. In this case, please look at the compile error and install the appropriate library (usually libxrandr etc.)

=== 3. Compile all the libraries.  ===
Compile the scl library and related 3rdparty libraries (Chai-graphics, yaml) all in one step

$ cd scl.git/applications-linux/scl_lib
# This is very slow
$ sh make_everything.sh
# This is much faster (but only compiles libraries; not executables)
$ sh make_libs.sh

=== 4. Explore the tutorials. ===
There are numerous tutorials designed to give you an idea of how scl works.
Please feel free to go through them. They should be self explanatory.

$ cd <top level scl dir>
$ cd scl_tutorial0_setup_robot
$ make release
$ ./scl_tutorial0_setup_robot

(or, for the more advanced tutorials)

$ cd <top level scl dir>
$ cd scl_tutorial3_graphics_physics_cmake
$ sh make_rel.sh
$ ./scl_tutorial3_graphics_physics_cmake

PS : Please go through tutorials in order.

=== 5. Explore the applications. ===
This will open the Pr2 robot with operational space control:

$ cd <top level scl dir>
$ cd applications_linux/scl_example_ctrl
$ sh make_rel.sh
$ ./scl_eg_ctrl ../../specs/Pr2/Pr2Cfg.xml Pr2Bot opc -ui hand2 -ui hand

You can move the robot's hands around using the 'w s a d q e' and 'u j h k y i' keys on the keyboard

Detailed installation instructions - see Getting started in our wiki:
https://bitbucket.org/samirmenon/scl-manips-v2/wiki/install/getting_started

==========================================================================================

Additional notes:

* For documentation:
$ cd <top level scl dir>/doc
$ evince StandardControlFramework-overview.pdf &

* For additional models:
$ cd <top level scl dir>/specs
- You can run different models by specifying the config file ../../specs/*/*Cfg.xml

* For additional controllers:
- You can look at the config files. The controllers are specified with xml.

* Writing your own controllers:
- Use the create new controller script in the applications-linux directory (follow instructions):
$ sh create_new_ctrl.sh
- Some applications use cmake. and for them use `mkdir build && cd build && cmake .. && make`
- Others use a makefile and you can build each one using `make release` or `make` (debug mode)

==========================================================================================


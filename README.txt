scl: Standard Control Library

A control and interactive simulation library. Supports torque control on a variety of robots including
Unimate PUMA 500, Barrett WAM, Kinova Jaco, Neurobot {v1, v2, v3}, HFI {v1, v2, v2.5, v3, v5}, Kuka LWR, 
and Kuka LBR IIWA. (Low-level robot drivers must be obtained separately).

Wiki: https://bitbucket.org/samirmenon/scl-manips-v2/wiki

==================================================================================================
== Getting started ==

=== 1.Install additional dependencies. ===
Open a terminal (ctrl+alt+t) and type:
$ sudo apt-get install build-essential cmake libusb-dev libpci-dev freeglut3-dev libncurses5-dev git-core gitk libusb-1.0-0-dev libglew-dev libqhull-dev libhiredis-dev libjsoncpp-dev libc6-dev-i386 redis-server redis-tools


===  2. Get the code. ===
I will assume you will get this on to your documents folder. Again, on the terminal:

$ cd ~/Documents
$ git clone https://<your-bitbucket-id>@bitbucket.org/samirmenon/scl-manips-v2.git scl-manips-v2.git

*** Note replace <your-bitbucket-id> with your actual bitbucket id..
*** Eg. $ git clone https://samirmenon@bitbucket.org/samirmenon/scl-manips-v2.git scl-manips-v2.git

=== Optional : Check out a stable version of the code ===
Rember, this step is __optional__. If a tag doesn't work for you, try using the latest branch (unstable in general but very likely to have your problem fixed).
Alternatively try an older tag or file some bugs.
$ git tag -l

Then pick whatever tag (stable point) you want. Preferably pick the most recent one.
$ git checkout -b tag-v0.99 tags/v0.99

Unless you have some experience coding, you should consider using a stable version of the code.

=== 3. Compile all the libraries.  ===
Compile the scl library and related 3rdparty libraries (Chai-graphics, yaml) all in one step

$ cd scl-manips-v2.git/applications-linux/scl_lib
$ sh make_everything.sh

=== 4. Explore the tutorials. ===
There are numerous tutorials designed to give you an idea of how scl works.
Please feel free to go through them. They should be self explanatory.

$ cd scl_tutorial0_setup_robot
$ make release
$ ./scl_tutorial0_setup_robot

(or, for the more advanced tutorials)

$ cd scl_tutorial3_graphics_physics_cmake
$ sh make_rel.sh
$ ./scl_tutorial3_graphics_physics_cmake

PS : Please go through tutorials in order.

=== 5. Explore the applications. ===
This will open the Pr2 robot with operational space control:

$ cd ../scl_example_ctrl
$ sh make_rel.sh
$ ./scl_eg_ctrl ../../specs/Pr2/Pr2Cfg.xml Pr2Bot opc -ui hand2 -ui hand

You can move the robot's hands around using the 'w s a d q e' and 'u j h k y i' keys on the keyboard

Detailed installation instructions - see Getting started in our wiki:
https://bitbucket.org/samirmenon/scl-manips-v2/wiki/install/getting_started

==========================================================================================

Additional notes:

* For documentation:
$ cd ~/scl-manips-v2.git/doc
$ evince StandardControlFramework-overview.pdf &

* For additional models:
$ cd ~/scl-manips-v2.git/specs
- You can run different models by specifying the config file ../../specs/*/*Cfg.xml

* For additional controllers:
- You can look at the config files. The controllers are specified with xml.

* Writing your own controllers:
- Use the create new controller script in the applications-linux directory (follow instructions):
$ sh create_new_ctrl.sh
- Some applications use cmake. and for them use `mkdir build && cd build && cmake .. && make`
- Others use a makefile and you can build each one using `make release` or `make` (debug mode)

==========================================================================================


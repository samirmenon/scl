scl: Standard Control Library

A control and interactive simulation library

Author: Samir Menon <smenon@stanford.edu>

Installation instructions:

1. Install additional dependencies. Open a terminal (ctrl+shift+t) and type:
$ sudo apt-get install build-essential cmake libusb-dev freeglut3-dev libncurses5-dev git-core gitk 

2. Get the code. I will assume you will get this on to your desktop. Again, on the terminal:
$ cd ~/Desktop
$ git clone https://<your-bitbucket-id>@bitbucket.org/samirmenon/scl-dev.git scl-dev.git

3. Compile the scl library and related 3rdparty libraries (Chai-graphics, yaml):
$ cd scl-dev.git/applications-linux/scl_lib
$ sh make_everything.sh

4. Explore the applications. This will open the Pr2 robot with operational space control
$ cd ../scl_example_ctrl
$ sh make_rel.sh
$ ./scl_eg_ctrl ../../specs/Pr2/Pr2Cfg.xml Pr2Bot opc hand hand2

==========================================================================================

Additional notes:

* For documentation:
$ cd ~/scl-dev.git/doc
$ evince StandardControlFramework-overview.pdf &

* For additional models:
$ cd ~/scl-dev.git/specs
- You can run different models by specifying the config file ../../specs/*/*Cfg.xml

* For additional controllers:
- You can look at the config files. The controllers are specified with xml.

* Writing your own controllers:
- Look at the applications-linux directory and copy paste the example controller
- Some applications use cmake. and for them use `mkdir build && cd build && cmake .. && make`
- Others use a makefile and you can build each one using `make release` or `make` (debug mode)

==========================================================================================

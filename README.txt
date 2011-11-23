scl: Standard Control Library

A control and interactive simulation library

Author: Samir Menon <smenon@stanford.edu>

Installation instructions:

Install additional dependencies: 
$sudo apt-get install build-essential cmake libusb-dev freeglut3-dev libncurses5-dev git-core gitk 

2. Compile the scl library and related 3rdparty libraries (Chai-graphics, yaml)

$ cd applications-linux/scl_lib
$ sh make_everything.sh

4. Explore the applications

$ cd ../scl_example_ctrl
$ sh make_rel.sh
$ ./scl_eg_ctrl ../../specs/Pr2/Pr2Cfg.xml Pr2Bot opc hand hand2

* You can run the models in ../../specs/*/*Cfg.xml
* Some apps use cmake. and for them use `mkdir build && cd build && cmake .. && make`
* Others use a makefile and you can build each one using `make release` or `make` (debug mode)

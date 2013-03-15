scl: Standard Control Library

A control and interactive simulation library

Author: Samir Menon <smenon@stanford.edu>

Wiki: https://bitbucket.org/samirmenon/scl-manips-group/wiki

==================================================================================================
== Getting started ==

=== 1.Install additional dependencies. ===
Open a terminal (ctrl+alt+t) and type:
$ sudo apt-get install build-essential cmake libusb-dev libpci-dev freeglut3-dev libncurses5-dev git-core gitk libusb-1.0-0


===  2. Get the code. ===
I will assume you will get this on to your documents folder. Again, on the terminal:

$ cd ~/Documents
$ git clone https://<your-bitbucket-id>@bitbucket.org/samirmenon/scl-manips-group.git scl-manips-group.git

=== 3. Compile all the libraries.  ===
Compile the scl library and related 3rdparty libraries (Chai-graphics, yaml) all in one step

$ cd scl-manips-group.git/applications-linux/scl_lib
$ sh make_everything.sh

=== 4. Explore the applications. ===
This will open the Pr2 robot with operational space control:

$ cd ../scl_example_ctrl
$ sh make_rel.sh
$ ./scl_eg_ctrl ../../specs/Pr2/Pr2Cfg.xml Pr2Bot opc -op hand2 -op hand


Up-to-date installation instructions - see Getting started in our wiki:
https://bitbucket.org/samirmenon/scl-manips-group/wiki/install/getting_started

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


Up-to date documentation you can find also in our wiki:
https://bitbucket.org/samirmenon/scl-manips-group/wiki/

==========================================================================================

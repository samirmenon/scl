scl: Standard Control Library

A control and interactive simulation library

Author: Samir Menon <smenon@stanford.edu>

Installation instructions:

1. Download Eigen 3.0.2
$ wget http://bitbucket.org/eigen/eigen/get/3.0.2.tar.gz 3rdparty/
$ cd 3rdparty
$ tar -xvzf 3.0.2.tar.gz 
$ move eigen* eigen3.0.2
$ cd ..

2. Compile Chai-graphics
$ cd 3rdparty/chai3d-graphics/lib/
$ sh make_debug.sh
$ sh make_release.sh
$ cd ../../../

3. Compile the scl library
$ cd applications-linux/scl_lib
$ sh make_debug.sh
$ sh make_release.sh
$ cd ..

4. Explore the applications
* You can build each one using `make release` or `make` (debug mode)
* You can run the models in ../../specs/*/*Cfg.xml


## BayesTracking - a library for Bayesian tracking

### Requirements
* Bayes++ (included in the repository)
* OpenCV 2.x (only to build the examples)

### Install (without catkin)
* Extract the library's compressed file
* In the library's directory, create a sub-directory `build`
* Enter in `./build` and run `cmake ..`
* Run `make` to build the library
* Run `make install` (as super user) to install the library and relative header files

### Install (ROS version)
* Run `rosdep` to resolve dependencies
* Run `catkin_make`
* Run `catkin_make install` to install the library and headers (optional)


_by Nicola Bellotto <nbellotto@lincoln.ac.uk>_

*cite with this DOI: [10.5281/zenodo.10318](http://dx.doi.org/10.5281/zenodo.10318)*

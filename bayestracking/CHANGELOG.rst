^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bayes_tracking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2014-09-03)
------------------
* Adding boost as a build an run dependency.
* Changing Licence to GPL
* Contributors: Christian Dondrup

1.0.5 (2014-10-29)
------------------
* Merge pull request `#9 <https://github.com/LCAS/bayestracking/issues/9>`_ from LCAS/indigo-fix
  Trusty/Indigo fix
* Trusty/Indigo fix
* Contributors: Christian Dondrup

1.0.4 (2014-10-29)
------------------
* Merge pull request `#8 <https://github.com/LCAS/bayestracking/issues/8>`_ from LCAS/opencv
  Changing dependency from opencv2 to cv_bridge for indigo release.
* Changing dependency from opencv2 to cv_bridge for indigo release.
* Contributors: Christian Dondrup

1.0.3 (2014-09-09)
------------------
* "Fixing" the missing JPDA
  The option of using only JPDA as an association algorithm was a relic from past development and is not supported anymore.
  This change just comments the JPDA in the enum, preventing people from trying to use it.
* Using the correct licence now.
* 1.0.2
* Updated changelog for 1.0.2
* Adding boost as a build an run dependency.
* 1.0.1
* Reverting the version number increase.
* Merge pull request `#5 <https://github.com/LCAS/bayestracking/issues/5>`_ from cdondrup/master
  Release preparations
* Contributors: Christian Dondrup

1.0.1 (2014-09-02)
------------------
* Merge pull request `#4 <https://github.com/cdondrup/bayestracking/issues/4>`_ from cdondrup/master
  Renamed `bayestracking` to `bayes_tracking` to comply with ros naming conventions.
* Renamed `bayestracking` to `bayes_tracking` to comply with ros naming conventions.
* Merge pull request `#2 <https://github.com/cdondrup/bayestracking/issues/2>`_ from cdondrup/master
  Adding optional catkin build
* * Enabling normal cmake build if catkin is not available.
  * Updated package.xml and create install targets for ROS release
  * Refactored README to markdown
* Refactored bayestracking library to be a catkin package
  Created a cleaner file structure by seperating the src from the include files.
* Update README
* Update README
* Now generates cmake config files to enable usage of find_package
* Added the creation of a pkg-config file to make the library easier to use.
* PFilter not working right now in simple_tracking.cpp, but all other tested filters (UKF and EKF) worked very well
* fixed size_t uint issue
* compiles now (but simple_tracking example still gives errors)
* initial import form package
* Contributors: Christian Dondrup, Marc Hanheide

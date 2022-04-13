### Overview

This folder contains the testing code for lidar obstacle detection and the PID controller for cruise control.

The organization of the project has changed since these tests were written, and so won't work out of the box. Furthermore,
the metrics the sample tests calculate for lidar are out of date. Tests are included here for reference 
of how tests can be written.

The general idea is to have an external program write value of interest to a file. For example, these values can
be the coefficent values for the controller. Then the program launches ROS, and in the node being tested, the
values of interest are read from the file and used in the logic. After the test has run, any quantities of interest
are written to a file. This second file is then read in by the external program and then processed to create a metric.

The reason for doing this is that it allows many tests to be run without having to manually launch them, and the 
tests can be easily relaunched after changes have been made. Note that a test is the external program mentioned earlier.

The tests are organized as C# tests, and can be run individually. The code that is likely to be of interest
are the files in the ControlFunctionality folder. These files control launching ROS and creating graphs.

Look at the Lidar or PIDController tests to see an example of how to lay out a test.

This is still not a very elegant solution overall. ROS does have some testing functionality in it, but at the
time when testing was performed, the documentation was not deemed to be adequate to be run. There are likely better
solutions to obtaining metrics in automated fashion.

lynxmotion_ssc32u_driver
========================

Package containing various controllers for the Lynxmotion SSC-32U servo controller using standard ROS message interfaces.

Overview
--------

Directly using the ssc32u_driver is not recommended, instead you should interface with a controller. This package contains a default set of controllers to meet most users needs. You may have one or more controllers running per SSC-32U device. This allows you to group the devices channels to be controlled independently from the other channels.

- **servo_controller** - Node using the JointTrajectory message to interface with the SSC-32U.
- **sabertooth_2x5_controller** - Node using the Twist message to interface with the SSC-32U.

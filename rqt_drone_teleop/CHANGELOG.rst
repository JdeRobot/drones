^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_drone_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.10 (2021-11-04)
-------------------

1.3.9 (2021-10-15)
------------------
* Added default values to ensure retro-compatibility
* Added takeoff config to rqt_pos_teleop
* Customizable takeoff height and precision from launch and during running
* RQT widgets are model dependant free.
* Added Tello compatibility basics
* Contributors: Diego Martín, Nikhil Khedekar, pariaspe

1.3.8 (2021-04-14)
------------------

1.3.7 (2021-03-01)
------------------

1.3.6 (2021-01-12)
------------------
* New setup.py. Fixing jenkins-build fail.
* Contributors: pariaspe

1.3.5 (2020-12-29)
------------------
* Freeze fixed using python threadings. Might be improved using a ROS-based solution.
* Removed RQT dependencies with subscribers and topics. These topics are now throttled by new drone_wrapper topics. This allows to make the RQT independent to the kind of drone connected. 
* Template dependency removed from rqt_drone_teleop. The template callbacks are no longer required to send commands to the drone. The command flow goes directly from the rqt to the drone wrapper without passing through the exercise template.
* Removed RQT dependencies with subscribers and topics. These topics are now throttled by new drone_wrapper topics. This allows to make the RQT independent to the kind of drone connected. 
* Template dependency removed from rqt_drone_teleop. The template callbacks are no longer required to send commands to the drone. The command flow goes directly from the rqt to the drone wrapper without passing through the exercise template.
* Contributors: pariaspe

1.3.4 (2020-06-28)
------------------
* Added perspectives
* READMEs updated
* Added new GUI screenshots
* Contributors: pariaspe

1.3.3 (2020-06-10)
------------------
* Takeoff button disabled until landed state is defined (drone is ready).
* Takeoff/land button synchronized between vel and pos teleop. 
* RQT freezing fixed.
* Drone teleop pkg modularized into two modules: rqt_vel_teleop and rqt_cam_viewer.
* One new module is added: rqt_pos_teleop.
* Added support to greyscale8 images.
* Contributors: pariaspe

1.3.2 (2020-05-06)
------------------
* New GUI for rqt_drone_teleop plugin. Added position and velocity info.
* Contributors: diegomrt, Pedro Arias

1.3.1 (2020-02-14)
------------------
* Fixing dependency on metapackage
* Contributors: Pedro Arias 

1.2.0 (2020-02-11)
------------------

1.1.0 (2020-01-20)
------------------
* generating pkg for melodic distro
* Contributors: Pedro Arias 

1.0.1 (2019-08-18)
------------------
* added topics as parameters
* Contributors: Nikhil Khedekar

1.0.0 (2019-08-01)
------------------
* corrected dependancies
* stop drone button now also stops code
* updated gui image
* removed launch and perspective
* Contributors: Nikhil Khedekar

0.0.1 (2019-07-20)
------------------
* updated makefile
* updated ui and functionality
* updated teleop
* added screenshot
* added mode 2 sticks
* changed casing and fixed image path for teleop
* corrected perspective
* shifted to a common teleop
* Contributors: Nikhil Khedekar

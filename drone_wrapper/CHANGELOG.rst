^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package drone_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.2 (2020-05-06)
------------------
* New velocity control. Masks modified. The drone now keeps it position when velocities are zero.
* Added position control.
* Added mixed control.
* Take off modified: now is done by mixed control. Time to take off reduced. Take off height can be configurable.
* Added all args to script. Needed to pass arguments to exercises.
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
* anonymised drone and added topics as parameters
* added new format launch files
* updated scripts and readme
* Contributors: Nikhil Khedekar

1.0.0 (2019-08-01)
------------------
* corrected dependancies
* Contributors: Nikhil Khedekar

0.0.1 (2019-07-20)
------------------
* updated makefile
* removed packages requiring source builds
* converted returns to numpy arrays
* made DroneWrapper importable
* Added files from colab-gsoc2019-Nikhil_Khedekar
* Contributors: Nikhil Khedekar

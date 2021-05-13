from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	packages=['rqt_cam_viewer', 'rqt_pos_teleop', 'rqt_vel_teleop'],
	package_dir={'': 'src'},
)

setup(**d)
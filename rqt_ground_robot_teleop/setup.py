from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	packages=['rqt_ground_robot_teleop'],
	package_dir={'': 'src'},
)

setup(**d)

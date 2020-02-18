## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['nightmare_control'],
    package_dir={'': '/home/nik/catkin_ws/src'},
    requires=['roslib', 'python_qt_binding', 'rospkg', 'rviz']
)

setup(**setup_args)
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['r2s2_for_rostest'],
    package_dir={'': 'nodes'},
    requires=['rospy']
)

setup(**d)

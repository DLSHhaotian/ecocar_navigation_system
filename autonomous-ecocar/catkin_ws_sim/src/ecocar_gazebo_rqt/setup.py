from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['ecocar_gazebo_rqt'],
    package_dir={'': 'src'},
    requires=['rospy']
)

setup(**d)
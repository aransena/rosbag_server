from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=['rosbag_server_classes'],
    package_dir={'' : 'src'},
    requires=['rosbag_server'])


setup(**setup_args)
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['o2as_easy_handeye'],
    scripts=[],
    package_dir={'': 'src/easy_handeye'})

setup(**setup_args)


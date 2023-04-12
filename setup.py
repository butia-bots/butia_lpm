from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=find_packages(where='src', exclude=[], include=['*']),
    package_dir={'': 'src'})

setup(**setup_args)
#!/home/zilong/anaconda3/envs/open3d_env/bin/python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup



# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['object_detection'],
    package_dir={'': 'src'}
)

setup(**setup_args)
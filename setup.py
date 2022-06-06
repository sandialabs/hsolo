#! /usr/bin/env python3
import os
import re
import sys
import sysconfig
import platform
import subprocess

from distutils.version import LooseVersion
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
from setuptools.command.test import test as TestCommand
from shutil import copyfile, copymode

use_prebuilt = False

# For WINDOWS environment, there are issues getting CMAKE building correctly under pip
# For now, we build the python libraries from the command line in Windows, and then set this flag to true
# and pip install will just copy the prebuild python binary libs
environment_variable_name = 'HSOLO_INSTALL_USE_PREBUILT'
environment_variable_value = os.environ.get( environment_variable_name, None )

if environment_variable_value is not None and environment_variable_value=='1':
    use_prebuilt = True

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def run(self):

        if use_prebuilt:
            return

        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: " +
                ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)',
                                         out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable,
                      '-DBUILD_STATIC_LIBRARY=OFF',
                      '-DBUILD_EXAMPLE_EXECUTABLE=OFF',
                      '-DBUILD_PYTHON_WRAPPER=ON',
                      '-DBUILD_MATLAB_MEX=OFF']

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(
                cfg.upper(),
                extdir)]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''),
            self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args,
                              cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args,
                              cwd=self.build_temp)

        print()  # Add an empty line for cleaner output


setup(
    name='pyhsolo',
    version='1.0',
    description='HSolo Homography Solver',
    long_description='',
    packages=find_packages(),
    install_requires=['numpy'],
    package_dir={'pyhsolo': 'pyhsolo'},
    ext_modules=[CMakeExtension('.')],
    cmdclass={'build_ext': CMakeBuild},
    package_data={'pyhsolo': ['*.so','*.pyd']},
    zip_safe=False,
)

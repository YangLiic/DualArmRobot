"""
Setup script for building Inovance Servo Python bindings

安装方法:
    pip install pybind11
    python setup.py build_ext --inplace
    
或者直接安装:
    pip install .
"""

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path"""
    def __str__(self):
        import pybind11
        return pybind11.get_include()

ext_modules = [
    Extension(
        'inovance_servo',
        sources=[
            'python_bindings.cpp',
            'inovance_servo.cpp',
            'hw_can_usb.cpp'
        ],
        include_dirs=[
            get_pybind_include(),
            '.',
        ],
        language='c++',
        extra_compile_args=['-std=c++14', '-pthread'],
        extra_link_args=['-pthread'],
    ),
]

setup(
    name='inovance_servo',
    version='1.0.0',
    author='Yang',
    description='Inovance Servo C++ SDK Python Bindings',
    long_description='Python bindings for Inovance Servo motor control using CANopen protocol',
    ext_modules=ext_modules,
    zip_safe=False,
    python_requires='>=3.6',
)

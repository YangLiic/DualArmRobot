from setuptools import setup, Extension
import pybind11
import os

# 获取 pybind11 的 include 路径
pybind11_include = pybind11.get_include()

cpp_dir = os.path.dirname(os.path.abspath(__file__))

ext_modules = [
    Extension(
        'cybergear_motor',
        sources=[
            os.path.join(cpp_dir, 'python_bindings.cpp'),
            os.path.join(cpp_dir, 'cybergear_motor.cpp'),
            os.path.join(cpp_dir, 'cybergear_can_usb.cpp'),
        ],
        include_dirs=[
            pybind11_include,
            cpp_dir,
        ],
        language='c++',
        extra_compile_args=['-std=c++14', '-O2', '-Wall'],
        extra_link_args=['-lpthread'],
    ),
]

setup(
    name='cybergear_motor',
    version='1.0.0',
    description='CyberGear 微电机 C++ 驱动 (Python 绑定)',
    ext_modules=ext_modules,
)

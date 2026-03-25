from setuptools import Extension, setup
import pybind11
import os

cpp_dir = os.path.dirname(os.path.abspath(__file__))

ext_modules = [
    Extension(
        "ze300_motor",
        sources=[
            os.path.join(cpp_dir, "python_bindings.cpp"),
            os.path.join(cpp_dir, "ze300_motor.cpp"),
            os.path.join(cpp_dir, "ze300_can_usb.cpp"),
        ],
        include_dirs=[
            pybind11.get_include(),
            cpp_dir,
        ],
        language="c++",
        extra_compile_args=["-std=c++14", "-O2", "-Wall"],
        extra_link_args=["-lpthread"],
    )
]

setup(
    name="ze300_motor",
    version="1.0.0",
    description="ZE300 motor C++ SDK (Python bindings)",
    ext_modules=ext_modules,
)

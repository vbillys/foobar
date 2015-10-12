from distutils.core import setup
from Cython.Build import cythonize

setup(
	name = 'RadarMsgsCython',
	ext_modules = cythonize("RadarMsgsCython.pyx"),
)

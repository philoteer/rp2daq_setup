#!/usr/bin/env python3
from setuptools import *

setup(
	name="rp2daq", 
	version='0.0.1',
	description='rp2daq',
	long_description='rp2daq, packaged.',
	packages=find_packages(),
	package_data={'': ['*.c','*.h']},
	install_requires=[]
)

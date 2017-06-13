#!/usr/bin/env python

from distutils.core import setup

setup(name='Emotive_Speech',
      version='1.0',
      description='Emotive Speech Transformation',
      author='Dereje Tadesse',
      author_email='dereje.tadesse@hansonrobotics.com',
      url='https://github.com/dergkat/emotivespeech',
      install_requires=[
      "pysptk >= 0.1.4",
      "sox  >= 1.2.9 "],
      packages=['emotivespeech'],
      license='Copyright 2017 Hanson Robotics. All rights reserved.'
     )

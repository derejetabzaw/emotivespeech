# Emotive Speech

A ROS package for emotive speech generation based on DAVID: An open-source platform for real-time emotional speech
transformation using pysox

Prerequisites
-------------
###### pysptk
> A python wrapper for Speech Signal Processing Toolkit (SPTK)
> https://github.com/r9y9/pysptk | sudo pip install pysptk

###### Basic Signal Processing Tools 
> Scipy,Numpy
> Make Sure scipy.io- dedicated wavefile read/write library- is also installed

###### pysox
> a Python library that provides a simple interface between Python and SoX.
> we will be using the Transformers (sox.transform.Transformers) for synthesis.
> http://pysox.readthedocs.io/en/latest/api.html | sudo apt-get install pysox



Build
-----
```sh
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ git clone https://github.com/dergkat/emotivespeech.git
$ cd ..
$ catkin_make
```

Run
---
```sh
roslaunch emo_speech emospeech.launch
```

###### rosnode
 * /emotrans

###### rosparam
 * /emotrans/chunk_size value: 1024
 * /emotrans/emotion_intensity: 100
 * /emotrans/file_names value: [ ]
 * /emotrans/sampling_frequency value: 8000


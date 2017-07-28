# Emotive Speech Python Package

## A Standard Python Package For Emotive Speech Generation Based on DAVID: An open-source platform for real-time emotional speech
transformation using pysox

Pre-Requisites
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

Installation and Setup
-----
##### After fulfilling all the requirements,The Python Package of the Emotive Speech Project can be cloned as:
```sh
$ git clone https://github.com/dergkat/emotivespeech.git
```
Usage
===========
##### 
```sh
$ cd ESPP/src
$ python -B EmotiveSpeech 
usage: emotivespeech.py [-h] [-c chunk_size] [-s semitones] [-r cutfreq]
                        [-g gain] [-q qfactor] [-v speed] [-d depth]
                        [-o tempo] [-i intensity] [-p parameter_control]
                        filename typeOfEmotion

```
Example
-----
```sh

$ python -B emotivespeech.py /home/user/Desktop/TestFolder/Test.wav sad
```
#### Arguments
```sh
$ python -B EmotiveSpeech.py arg1 arg2
```
###### arg1(Positional Argument): Absolute Path For Wavefile
###### arg2(Positional Argument): TypeofEmotion: (happy,sad,afraid,happy_tensed) 




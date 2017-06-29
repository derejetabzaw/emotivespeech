# Emotive Speech

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
##### After fulfilling all the requirements,The Python Package of the Emotive_Speech Project can be cloned as:
```sh
$ git clone https://github.com/dergkat/emotivespeech/tree/python_package
```
Basic Usage
===========
##### An Example of the Package can be seen below:
##### Open `EmotiveSpeech.py`, the Caller Function

```python

FILE_NAME_PATH = "/specify/abs/path/for/the/wavefile"
CHUNK_SIZE = 1024	  
typeOfEmotion = "happy"
		
		
		

def emotive_speech(FILE_NAME_PATH,CHUNK_SIZE,typeOfEmotion):
	
	"""
	#Computation is done here
	"""
	return output

if __name__ == '__main__':	
	emotive_speech(FILE_NAME_PATH,CHUNK_SIZE,typeOfEmotion)
```
##### OR:
##### From the terminal you can use it as:

```sh
$ cd python_package/src
$ python -B EmotiveSpeech /abs/path/ int(chunk_size) typeofEmotion
```

Example
-----
```sh
$ cd python_package/src
$ python -B EmotiveSpeech /home/user/Desktop/TestFolder/Test.wav 1024 sad
```
#### Arguments

```sh
$ python -B EmotiveSpeech.py arg1 arg2 arg3
```
###### arg1: Absolute Path For Wavefile

###### arg2: Chunk_Size(256,512,1024,2048,4096...)

###### arg3: TypeofEmotion: (happy,sad,afraid,happy_tensed) 




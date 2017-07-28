.. Emotive Speech documentation master file, created by
   specifieshinx-quickstart on Fri Jun 23 11:42:21 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Emotive Speech's documentation!
====================================

A Standard Python Package For Emotive Speech Generation Based on `DAVID <http://cream.ircam.fr/?p=44>`_ : An open-source platform for real-time emotional speech

Pre-Requisites
====================

**pysptk**
A python wrapper for Speech Signal Processing Toolkit (SPTK)

Click `this <https://github.com/r9y9/pysptk>`_ link or ``sudo pip install pysptk``

**Basic Signal Processing Tools**

* Scipy,Numpy
* Make Sure scipy.io- dedicated wavefile read/write library- is also installed

**pysox**
A Python library that provides a simple interface between Python and SoX.
We will be using the Transformers (sox.transform.Transformers) for synthesis.
Check `this <http://pysox.readthedocs.io/en/latest/api.html>`_ | ``sudo apt-get install pysox``


**Installation and Setup**

*After fulfilling all the requirements,The Python Package of the Emotive Speech Project can be cloned as:*

``$ git clone https://github.com/dergkat/emotivespeech.git``


.. toctree::
   :maxdepth: 2

Modules and Functions
============================

.. py:module:: EmotiveSpeech

.. py:function:: emotive_speech(fname,ofile,chunk_size,typeOfEmotion,semitones,gain,qfactor,speed,depth,tempo,intensity,parameter_control)

   A caller module

   :param str fname: input file path
   :param str ofile: output file path
   :param int chunk_size:
   :param str typeOfEmotion: The type of selected emotion
   :param float semitone: 
   :param float gain:
   :param int qfactor:
   :param float speed:
   :param float depth:
   :param float tempo:
   :param float intensity:
   :param float parameter_control:
   :return: output
   :rtype: object


.. py:module:: analysis

.. py:function:: pitch_detect(sndarray,fs, chunk_size)

 	pitch_detect computes the fundamental frequency/pitches of blocks/ of Chunks

 	:param array sndarray: Discrete Data
 	:param int fs: Sampling frequency
 	:param int chunk_size: Chunk size
 	:return: f0
 	:rtype: int


.. py:function:: zero_crossing_rate_blocks(wavedata,Chunk_Size,fs)

	This module computes the zero crossing rate of blocks.
	It is important in classifying/detecting the voiced and unvoiced region.
	If the ZCR is smaller then it is categorized as voiced.
	If the ZCR is greater then it is categorized as unvoiced.
	Click: `this <https://www.asee.org/documents/zones/zone1/2008/student/ASEE12008_0044_paper.pdf>`_ for more info
		
	
	:param str wavedate: Discrete Data
 	:param int fs: Sampling frequency
 	:param int chunk_size: Chunk size
 	:return: zero_crossing_rate_obj
 	:rtype: obj


.. py:function:: root_mean_square(x,Chunk_Size,fs)

 	root_mean_square computes the root mean square of the blocks and is important for categorizing inflection pitch-bending samples

 	:param str x: 
 	:param int fs: Sampling frequency
 	:param int chunk_size: Chunk size
 	:return: rms-root
 	:rtype: int

.. py:function:: starting_info(x,f0,fs,Chunk_Size)

	starting_info specifies the voiced and unvoiced starting points.
	It also contains the samples of the voiced and unvoiced samples.
	It will help,mainly,to get the voiced_samples.

 	:param str x: Discrete Data
 	:param int f0: Fundamental frequency
 	:param int fs: Sampling Frequency
 	:param int Chunk_Size: Chunk Size
 	:return: voiced_unvoiced_starting_info_object- Contains object of the unvoicedStart,VoicedStart,unvoicedSamples('USamp') and voicedSamples(VSamp)
 	:rtype: obj_array

.. py:module:: batchprocess

.. py:function:: process_variables(x,fs,chunk_size)

 	computes basic variables important for the entire process.

 	:param obj x: discrete data from the wavefile
 	:param int fs: Sampling frequency
 	:param int chunk_size: The size of block containing datas
 	:return: Time Stamps-These are the time stamps in secs sampled at fs
			 Consecutive Blocks- These are consecutive blocks that are important for inflection and/or pitch bending	
 	:rtype: obj


.. py:function:: batch_analysis(x,fs,chunk_size)

 	computes the fundamental frequency/pitch of blocks/,voiced_samples and the rms values that are important for analysis and will be used for pre-process

 	:param obj x: discrete data from the wavefile
 	:param int fs: Sampling frequency
 	:param int chunk_size: The size of block containing datas
 	:return: fundamental_frequency_in_blocks- This is a fundamental frequency(or pitch) for the blocks in Chunk_Size
					
			voiced_samples-These are samples that contain the voiced samples.Will be usedfor the entire process and is important for the synthesis process as well.
					
			rms- is the root mean square computation that will be important for ategorizing inflecion/pitch bending samples.
 	:rtype: obj

.. py:function:: batch_preprocess(fundamental_frequency_in_blocks,voiced_samples,rms)

 	This is the pre-process or pre-synthesis stage. This module computes the samples for the begining of utterances and finally computes the selected_inflect_block 

 	:param int fundamental_frequency_in_blocks: This is a fundamental frequency(or pitch) for the blocks in Chunk_Size
 	:param obj voiced_samples: These are samples that contain the voiced samples. 
 	:param obj rms: This is the root mean square computation
 	:return: selected_inflect_block- are the blocks that are important for the synthesis process
 	:rtype: obj

.. py:function:: batch_synthesis(fs,consecutive_blocks,time_stamps,selected_inflect_block_new,typeOfEmotion,semitones,cutfreq,gain,qfactor,speed,depth,tempo,intensity,parameter_control)

 	This is the synthesis stage. This modules gives emotions of "Happy","Happy-Tensed","Sad","Afraid" for the wavefile using the process variables and selected_inflect_blocks

    
    :param int fs: Sampling frequency
    :param obj consecutive_blocks: 
    :param obj time_stamps:
    :param obj selected_inflect_block_new:
    :param str typeOfEmotion:
    :param float semitones:
    :param int cutfreq:
    :param float gain:
    :param int qfactor:
    :param float speed:
    :param float depth:
    :param float tempo:
    :param float intensity:
    :param float parameter_control:
    :return: output- Modified/Synthesised wavefile
    :rtype: obj

.. py:module:: preprocess

.. py:function:: utterance_region_begin_samples(voiced_samples)

 	utterance_region_begin_samples(voiced_samples) utterance_region_begin_samples - This module help in finding the begining of voice samples and its respective index.

 	:param obj voiced_samples:
 	:return: voice_sample_begin-Contains array that contain the beginning voiced samples voice_sample_index-Contains array that contain the index of the voiced_sample_begin
 	:rtype: obj

.. py:function:: utterance_chunk(voiced_samples,voice_sample_index)

 	utterance_chunk:is a module used to get the voiced_chunk_samples. The voiced_chunk_samples contain voice samples that are bundled together.

 	Eg. [1,2][6,7,8,9,10][34,35,36],[45,46,47,48] This wavefile has Four Utterance Chunks The voiced and unvoiced samples currently use zero crossing rate(zcr) to classify regions.

 	:param obj voiced_samples:
 	:param int voice_sample_index:
 	:return: voiced_chunk_samples
 	:rtype: obj


.. py:function:: pre_process(Voiced_Samples)


 	:param obj Voiced_Samples:
 	:return: inflection_voices_samples - samples of the inflection voiced sample
 	:rtype: obj

.. py:function:: matrix_of_sample_numbers(rms,inflection_voices_samples)

 	[inflect_sample] This matrix contains sample numbers for a difference of voiced samples that are less than the mean of the rms values. According to D.A.V.I.D, this samples are recorded as attack Since we will be applying inflection on new utterance, This will help for classifications amongst voiced samples. 

 	:param int rms:
 	:param obj inflection_voice_samples:
 	:return: inflect samples
 	:rtype: obj

.. _selected_inflect_block_new:

.. py:function:: selected_inflect_block_new(inflection_sample_numbers)

 	This module is used for computing the selected inflect blocks 

 	:param int inflection_sample_numbers: are the whole samples that are potential sample numbers for inflection
 	:return: selected_inflect_block - are the selected inflect blocks that will be helpful for the entire synthsis properties 
 	:rtype: obj

.. py:module:: synthesis


.. _appended_utterance_time_stamps:

.. py:function:: appended_utterance_time_stamps(consecutive_blocks, time_stamps, selected_inflect_block)

 	This is appended time stamps for particular utterances. It is important for the synthesis process. It is time stamps of the utternace region we have got from the selected_infect_block. 
	
	See: selected_inflect_block_new_.

 	:param int consecutive_blocks:
 	:param obj time_stamps:
 	:param obj selected_inflect_block:
 	:return: utterance_time_stamps
 	:rtype: obj

.. _happy_inflection_function:

.. py:function:: happy_inflection_function(normalized_time_stamps)

 	This module is a function that accepts time stamps and outputs a cent. The function is made using polynomial fit with a 4th-Degree polynomial. The particular seconds and the corresponding cents are provided from the D.A.V.I.D Windows platform. You can download the application on windows It is an open source Follow `this <https://cycling74.com/downloads/older/>`_ link

 	:param obj normalized_time_stamps:
 	:return: cents- These are helpful in creating inflections
 	:rtype: obj

.. _afraid_inflection_function:

.. py:function:: afraid_inflection_function(normalized_time_stamps)

 	See: happy_inflection_function_.

 	:param obj normalized_time_stamps:
 	:return: cents- These are helpful in creating inflections for afraid_patch
 	:rtype: obj

.. py:function:: happy_tensed_inflection_function(normalized_time_stamps)

 	same as happy_inflection_function(normalized_time_stamps) except for specification for the 4th Degree polynomial fitting. 
 	
 	See: happy_inflection_function_.

 	:param obj normalized_time_stamps:
 	:return: cents- These are helpful in creating inflections
 	:rtype: obj

.. py:function:: normalize_function(utterance_time_stamps)

 	This module normalizes the utterance time-stamps within the bounded time range of the total inflect duration(500ms/0.5s). 
	
	See: appended_utterance_time_stamps_.

 	:param obj utterance_time_stamps:
 	:return: normalized_utterance
 	:rtype: obj


.. py:function:: happy_patch(sampleFrequency,utterance_begin)

 	The module helps to synthesis the "Happy" Emotion using the picth shift, picth bend(inflection) and filtering. 
 	
 	See: appended_utterance_time_stamps_.

 	:param int sampleFrequency:
 	:param obj utterance_begin:
 	:return: happy_patch
 	:rtype: obj

.. py:function:: happy_tensed_patch(sampleFrequency,utterance_begin)

 	The module helps to synthesis the "Happy_Tensed" Emotion using the picth shift, picth bend(inflection) and filtering.   
 	
 	See: appended_utterance_time_stamps_.

 	:param int sampleFrequency:
 	:param obj utterance_begin:
 	:return: happy_tensed_patch
 	:rtype: obj


.. py:function:: sad_patch(sampleFrequency,utterance_begin)

 	The module helps to synthesis the "Sad" Emotion using the picth shift and through filtering

 	:param int sampleFrequency:
 	:return: sad_patch
 	:rtype: obj

.. py:function:: afraid_patch(sampleFrequency,utterance_begin)

 	The module helps to synthesis the "Afraid" Emotion using the picth bend(inflection) and tremelo.   
 	
 	See: appended_utterance_time_stamps_.

 	:param int sampleFrequency:
 	:param obj utterance_begin:
 	:return: afraid_patch
 	:rtype: obj

.. py:module:: Example

Usage and Examples
==================

Usage
##########


``$ cd ESPP/src``

``$ python -B EmotiveSpeech``

to get help: emotivespeech.py [-h]



Example
###########

``$ python -B emotivespeech.py /home/user/Desktop/TestFolder/Test.wav sad``

``$ python -B EmotiveSpeech.py arg1 arg2``

* arg1(Positional Argument): Absolute Path For Wavefile
* arg2(Positional Argument): TypeofEmotion: (happy,sad,afraid,happy_tensed) 

Testing files are available `here <https://github.com/dergkat/emotivespeech/tree/master/ESPP/examples/files>`_




Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


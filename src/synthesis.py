from scipy.signal import butter
import numpy as np
from sox.transform import Transformer
import rospy

FILE_NAME_PATH = ''
CUTFREQ = 4000
QFACTOR = 1 
PARAMETER_CONTROL = 1
output_file_path=""

def appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block):
	"""
	appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
			This is appended time stamps for particular utterances.
			It is important for the synthesis process. It is time stamps
			of the utternace region we have got from the selected_infect_block
				
		Parameter:CONSECUTIVE_BLOCKS
				  TIME_STAMPS
				  selected_inflect_block
		
		Returns: utterance_time_stamps 
	
	See: selected_inflect_block_new(inflection_sample_numbers)
	"""

	utterance_time_stamps = []
	for i in range(len(selected_inflect_block)):
		utterance_time_stamps.append(TIME_STAMPS[selected_inflect_block[i][:CONSECUTIVE_BLOCKS]])
	utterance_time_stamps = np.asarray(utterance_time_stamps)
	return utterance_time_stamps
def happy_inflection_function(normalized_time_stamps):
	"""
	happy_inflection_function(normalized_time_stamps)
			This module is a function that accepts time stamps and outputs a cent.
			The function is made using polynomial fit with a 4th-Degree polynomial
			The particular seconds and the corresponding cents are provided from 
			the D.A.V.I.D Windows platform 
		
		Parameter: normalized_time_stamps
		Returns: cents- This are helpful in creating inflections
	
	See: You can download the application on windows
		 It is an open source
		 https://cycling74.com/downloads/older/
	"""

	time = np.array([0.01,0.058511,0.255319,0.401596,0.500],ndmin=1)
	fixed_cents = np.array([-200,140,82.667,0.001,0.001],ndmin=1)

	func = np.polyfit(time,fixed_cents,4)
	inflect_func = np.poly1d(func)
	cents = inflect_func(normalized_time_stamps)
	return cents.tolist()
def afraid_inflection_function(normalized_time_stamps):
	"""
	afraid_inflection_function(normalized_time_stamps)
		Parameter: normalized_time_stamps
		Returns:   cents- This are helpful in creating inflections for afraid_patch
	
	See: happy_inflection_function(normalized_time_stamps)
	"""

	time = np.array([0.01,0.058511,0.255319,0.401596,0.500],ndmin=1)
	fixed_cents = np.array([120,0.01,8,192,-192],ndmin=1)
	func = np.polyfit(time,fixed_cents,4)
	inflect_func = np.poly1d(func)
	cents = inflect_func(normalized_time_stamps)
	return cents.tolist()
def happy_tensed_inflection_function(normalized_time_stamps):
	"""
	happy_tensed_inflection_function(normalized_time_stamps)
			
			same as happy_inflection_function(normalized_time_stamps)
			except for specification for the 4th Degree polynomial fitting
		
		Parameter: normalized_time_stamps
		Returns:   cents- This are helpful in creating inflections
	
	See: happy_inflection_function(normalized_time_stamps)
	"""
	time = np.array([0.01,0.058511,0.255319,0.401596,0.500],ndmin=1)
	fixed_cents = np.array([0.001,0.001,82.667,-82.667,0.001],ndmin=1)
	func = np.polyfit(time,fixed_cents,4)
	inflect_func = np.poly1d(func)
	cents = inflect_func(normalized_time_stamps)
	return cents.tolist()
def normalize_function(utterance_time_stamps):
	"""
	normalize_function(utterance_time_stamps)
			
			This module normalizes the utterance time-stamps within 
			the bounded time range of the total inflect duration(500ms/0.5s) 
		Parameter: utterance_time_stamps
		
		Returns: normalized_utterance
	
	See: appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
		 
	"""

	normal = max(utterance_time_stamps) - min(utterance_time_stamps)
	utterance_time_stamps = (utterance_time_stamps-min(utterance_time_stamps))/normal
	normal_two = 0.5 - 0.01
	normailzed_utterance  = (utterance_time_stamps * normal_two) + 0.01
	return normailzed_utterance 
def start_end_times(utterance_begin):
	start_time_now = []
	end_time_now = []
	for i in range(len(utterance_begin)):
		start_time_now.append(utterance_begin[i][:-1])
		end_time_now.append(utterance_begin[i][1:])
	start_time_now = np.asarray(start_time_now)
	end_time_now = np.asarray(end_time_now)
	return start_time_now,end_time_now
def happy_cents_for_utterance(start_time_now):
	cents = []
	for i in range(len(start_time_now)):
		cents.append(happy_inflection_function(normalize_function(start_time_now[i])))
	cents = np.concatenate(cents)
	cents = cents.tolist()
	return cents
def afraid_cents_for_utterance(start_time_now):
	cents = []
	for i in range(len(start_time_now)):
		cents.append(afraid_inflection_function(normalize_function(start_time_now[i])))
	cents = np.concatenate(cents)
	cents = cents.tolist()
	return cents
def happy_tensed_cents_for_utterance(start_time_now):
	cents = []
	for i in range(len(start_time_now)):
		cents.append(happy_tensed_inflection_function(normalize_function(start_time_now[i])))
	cents = np.concatenate(cents)
	cents = cents.tolist()
	return cents
def concatenate_list(start_time_now,end_time_now):
	start_time_now = np.concatenate(start_time_now)
	end_time_now = np.concatenate(end_time_now)
	start_time_now = start_time_now.tolist()
	end_time_now = end_time_now.tolist()
	return start_time_now,end_time_now

def happy_sox_init(output_file_path,semitones,number_of_bends,start_time_now,end_time_now,cents,CUTFREQ,gain,QFACTOR):
	patch = Transformer()
	patch.pitch(semitones,False)
	patch.tempo(1.1,'s')
	patch.gain(2.0)
	#patch.bend(number_of_bends,start_time_now,end_time_now,cents,50)			
	patch.treble(gain,CUTFREQ,0.5)
	patch.equalizer(CUTFREQ,QFACTOR,gain)
	print FILE_NAME_PATH
	print output_file_path
	patch.build(FILE_NAME_PATH,output_file_path)
	return patch
def afraid_sox_init(speed,depth,number_of_bends,start_time_now,end_time_now,cents,output_file_path):
	patch = Transformer()
	patch.tremolo(speed,depth)
	patch.tempo(1.05,'s')
	patch.gain(1.1)
	#patch.bend(number_of_bends,start_time_now,end_time_now,cents,50)			
	patch.build(FILE_NAME_PATH,output_file_path)
	return patch
def happy_tensed_sox_init(output_file_path,semitones,number_of_bends,start_time_now,end_time_now,cents,CUTFREQ,gain,QFACTOR):
	patch = Transformer()
	patch.pitch(semitones,False)
	patch.tempo(1.18,'s')
	patch.gain(gain)
	#patch.bend(number_of_bends,start_time_now,end_time_now,cents,50)			
	patch.treble(gain,CUTFREQ,0.5)
	patch.equalizer(CUTFREQ,QFACTOR,gain)
	patch.build(FILE_NAME_PATH,output_file_path)
	return patch
def sad_sox_init(semitones,gain,CUTFREQ,output_file_path):
	CUTFREQ = 3500
	patch = Transformer()
	patch.pitch(semitones,False)
	patch.tempo(0.95,'s')
	patch.treble(gain,CUTFREQ,0.5)
	patch.build(FILE_NAME_PATH,output_file_path)
	return patch


def happy_patch(sampleFrequency,utterance_begin):
	"""
	happy_patch(sampleFrequency,utterance_begin)
			The module helps to synthesis the "Happy" Emotion 
			using the picth shift, picth bend(inflection) and filtering
		
		Parameter: sampleFrequency
				   utterance_begin 
		
		Return:	   happy_patch	
	
	See: appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
	"""

	gain = 3.0
	semitones = 1.5 * PARAMETER_CONTROL
	start_time_now,end_time_now=start_end_times(utterance_begin)
	cents = happy_cents_for_utterance(start_time_now)
	start_time_now,end_time_now=concatenate_list(start_time_now,end_time_now)
	number_of_bends = len(start_time_now)
	happy_patch = happy_sox_init(output_file_path,semitones,number_of_bends,start_time_now,end_time_now,cents,CUTFREQ,gain,QFACTOR)
	return happy_patch

def happy_tensed_patch(sampleFrequency,utterance_begin):
	"""
	happy_tensed_patch(sampleFrequency,utterance_begin)
			The module helps to synthesis the "Happy_Tensed" Emotion 
			using the picth shift, picth bend(inflection) and filtering
		
		Parameter: sampleFrequency
				   utterance_begin 
		
		Return:	   happy_tensed_patch	
	
	See: appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
	
	"""


	gain = 3.0
	semitones = 2.0 * PARAMETER_CONTROL
	start_time_now,end_time_now=start_end_times(utterance_begin)
	cents = happy_tensed_cents_for_utterance(start_time_now)
	start_time_now,end_time_now=concatenate_list(start_time_now,end_time_now)
	number_of_bends = len(start_time_now)
	happy_tensed_patch = happy_tensed_sox_init(output_file_path,semitones,number_of_bends,start_time_now,end_time_now,cents,CUTFREQ,gain,QFACTOR)
	return happy_tensed_patch


def sad_patch(sampleFrequency):
	"""
	sad_patch(sampleFrequency,utterance_begin)
			The module helps to synthesis the "Sad" Emotion 
			using the picth shift and through filtering
		
		Parameter: sampleFrequency
		
		Return:	   sad_patch	
	"""


	gain = 0.25
	semitones = -1.5 * PARAMETER_CONTROL
	sad_patch = sad_sox_init(semitones,gain,CUTFREQ,output_file_path)
	return sad_patch


def afraid_patch(sampleFrequency,utterance_begin):
	"""
	afraid_patch(sampleFrequency,utterance_begin)
			The module helps to synthesis the "Afraid" Emotion 
			using the picth bend(inflection) and tremelo
		
		Parameter: sampleFrequency
				   utterance_begin 
		
		Return:	   afraid_patch	
	
	See: appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
	
	"""



	speed = 8.5
	depth = 1 + (60 * PARAMETER_CONTROL)
	start_time_now,end_time_now=start_end_times(utterance_begin)
	cents = afraid_cents_for_utterance(start_time_now)
	start_time_now,end_time_now = concatenate_list(start_time_now,end_time_now)
	number_of_bends = len(start_time_now)
	afraid_patch = afraid_sox_init(speed,depth,number_of_bends,start_time_now,end_time_now,cents,output_file_path)
	return afraid_patch


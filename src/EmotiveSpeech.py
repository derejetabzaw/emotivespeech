import numpy as np
import preprocess as prep
import warnings
warnings.filterwarnings('ignore')
import batchprocess as bp
import synthesis as syn
import rospy
import os

CHUNK_SIZE = 1024


def emotive_speech(x,fs,typeOfEmotion):
	"""
	emotive_speech(x,fs,typeOfEmotion)
			A Caller Module
		Parameter:x
				  fs
				  typeOfEmotion
		Returns: output
	"""
	
	TIME_STAMPS = bp.process_variables(x,fs,CHUNK_SIZE)[0]
	CONSECUTIVE_BLOCKS = bp.process_variables(x,fs,CHUNK_SIZE)[1]
	fundamental_frequency_in_blocks = bp.batch_analysis(x,fs,CHUNK_SIZE)[0]
	voiced_samples = bp.batch_analysis(x,fs,CHUNK_SIZE)[1]
	rms = bp.batch_analysis(x,fs,CHUNK_SIZE)[2]
	selected_inflect_block = bp.batch_preprocess(fundamental_frequency_in_blocks,voiced_samples,rms)
	output = bp.batch_synthesis(fs,CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block,typeOfEmotion)
	return output

def generate_file(emotion):
	for i in rospy.get_param('file_names'):
		syn.FILE_NAME_PATH = i
		syn.output_file_path = os.path.dirname(i) + "/__generated_" + rospy.get_param('selected_emotion') + '_' + os.path.basename(i)
		fs,x = prep.wave_file_read(i)
		emotive_speech(x, fs, rospy.get_param('selected_emotion'))


	# if emotion=='happy':
	# 	emotive_speech(x,fs,"Happy")
	# elif emotion=='happy_tensed':
	# 	emotive_speech(x,fs,"HappyTensed")
	# elif emotion=='sad':
	# 	emotive_speech(x,fs,"Sad")
	# elif emotion=='afraid':
	# 	emotive_speech(x,fs,"Afraid")

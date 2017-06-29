import numpy as np
import analysis as alysis
import preprocess as prep
import synthesis as synth

def process_variables(x,fs,CHUNK_SIZE):
	"""
	process_variables(x,fs,CHUNK_SIZE)
			computes basic variables important for the entire process.
		Parameters: x-discrete data from the wavefile
					fs-sampling frequency
					Chunk_Size- The size of block containing datas
		
		Returns: 	Time Stamps-This are the time stamps in secs sampled at fs
					Consecutive Blocks- This are consecutive blocks that are important 
					for inflection and/or pitch bending			  
	"""
	NUM_BLOCKS = int(np.ceil(len(x)/CHUNK_SIZE))
	SAMPLE_PERIOD = 1/float(fs) * CHUNK_SIZE
	
	TIME_STAMPS = (np.arange(0,NUM_BLOCKS -1)* (CHUNK_SIZE/float(fs)))
	CONSECUTIVE_BLOCKS = 1 + int(0.5 / SAMPLE_PERIOD)
	return TIME_STAMPS,CONSECUTIVE_BLOCKS
def batch_analysis(x,fs,CHUNK_SIZE):
	"""
	batch_analysis(x,fs,CHUNK_SIZE)
	
			computes the fundamental frequency/pitch of blocks/,voiced_samples and the rms values 
			that are important for analysis and will be used for pre-process
		Parameters:  x-discrete data from the wavefile
					 fs-sampling frequency
					 Chunk_Size- The size of block containing datas
		
		Returns:	fundamental_frequency_in_blocks- This is a fundamental frequency(or pitch) 
					for the blocks in Chunk_Size
					
					voiced_samples-This are samples that contain the voiced samples.Will be used
				    for the entire process and is important for the synthesis process as well.
					
					rms- is the root mean square computation that will be important for 
					categorizing inflecion/pitch bending samples.
	"""

	fundamental_frequency_in_blocks = alysis.pitch_detect(x,fs,CHUNK_SIZE)
	rms = alysis.root_mean_square(x,CHUNK_SIZE,fs)
	voiced_unvoiced_starting_info_object = alysis.starting_info(x,fundamental_frequency_in_blocks,fs,CHUNK_SIZE)
	voiced_samples = voiced_unvoiced_starting_info_object['VSamp']
	return fundamental_frequency_in_blocks,voiced_samples,rms
def batch_preprocess(fundamental_frequency_in_blocks,voiced_samples,rms):
	"""
	batch_preprocess(fundamental_frequency_in_blocks,voiced_samples,rms)
	
			This is the pre-process or pre-synthesis stage. This module computes the
			samples for the begining of utterances and finally computes the selected_inflect_block 
		Parameters: fundamental_frequency_in_blocks-This is a fundamental frequency(or pitch) 
					for the blocks in Chunk_Size
					voiced_samples-This are samples that contain the voiced samples.
					rms-is the root mean square computation
		Returns:	selected_inflect_block- are the blocks that are important for the synthesis process
	"""

	voice_sample_begin = prep.utterance_region_begin_samples(voiced_samples)
  	voice_chunk_sample = prep.utterance_chunk(voiced_samples,voice_sample_begin[1])
	inflection_voice_samples = prep.pre_process(voice_chunk_sample)
	#frequency_of_voiced_samples = fundamental_frequency_in_blocks[voiced_samples]
	#frequency_for_inflection = prep.potential_inflection_fundamental_frequency(frequency_of_voiced_samples)
	inflection_sample_numbers = prep.matrix_of_sample_numbers(rms[voice_sample_begin[0]],inflection_voice_samples)
	selected_inflect_block= prep.selected_inflect_block_new(inflection_sample_numbers)
	return selected_inflect_block
def batch_synthesis(fs,CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block_new,typeOfEmotion):
	"""
	batch_synthesis(fs,CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block_new,typeOfEmotion)
	
			This is the synthesis stage. This modules gives emotions of 
			"Happy","Happy-Tensed","Sad","Afraid" for the wavefile using 
			the process variables and selected_inflect_blocks
			
		Parameters: fs
					CONSECUTIVE_BLOCKS
					TIME_STAMPS
					selected_inflect_block_new
					typeOfEmotion
		Returns: 	output- Modified/Synthesised wavefile
	"""
	

	if typeOfEmotion == "happy":
		selected_inflect_block = selected_inflect_block_new
		utterance_time_stamps = synth.appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
		output = synth.happy_patch(fs,utterance_time_stamps)	
	
	if typeOfEmotion == "happy_tensed":	
		selected_inflect_block = selected_inflect_block_new
		utterance_time_stamps = synth.appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
		output = synth.happy_tensed_patch(fs,utterance_time_stamps)

	if typeOfEmotion == "sad":	
		output = synth.sad_patch(fs)
	
	if typeOfEmotion == "afraid":
		selected_inflect_block = selected_inflect_block_new
		utterance_time_stamps = synth.appended_utterance_time_stamps(CONSECUTIVE_BLOCKS,TIME_STAMPS,selected_inflect_block)
		output = synth.afraid_patch(fs,utterance_time_stamps)
	return output

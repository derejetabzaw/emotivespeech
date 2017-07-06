import unittest
import numpy as np
from scipy.io import wavfile
import pysptk
import os
from os.path import join
import sys
cwd = os.path.dirname(os.path.realpath(__file__))
_root = os.path.split(cwd)[0]
sys.path.append(_root)
import src.preprocess as prep
import src.analysis as alysis

test_case_voice_samples = np.array([4,5,6,9,10,11,17,18,20,21])
test_case_voice_samples_two = np.array([[4,5,6,7,8],[9,10,11],[17,18,19,20,21,22],[24,25,26]],dtype=object)
test_data = np.array([52,20,4,9,8,7,10,31,22,34,27,11,13,72,65])
test_chunk_size = 3
test_sampling_frequency = 44100
test_RMS = np.array([39.39543121,6.7251,8.51469318,23.03258561,30.70016287,24.7994,64.2145,21.46,12.6667,13.5478])

class TestPreProcess(unittest.TestCase):

 	def test_utterance_region_begin_samples(self):
 		expected_begin_voice_samples = np.array([4,9,17,20])
 		expected_begin_voice_samples_index = np.array([0,1,2,3])
 		actual_begin_voice_samples,actual_begin_voice_samples_index = prep.utterance_region_begin_samples(test_case_voice_samples)	
 		self.assertItemsEqual(expected_begin_voice_samples,actual_begin_voice_samples)
 	def test_utterance_chunk(self):
		expected_utterance_chunk = [4,5,6],[9,10,11],[17,18],[20,21]
		actual_begin_voice_samples_index = prep.utterance_region_begin_samples(test_case_voice_samples)[1]
		actual_utterance_chunk = np.array(prep.utterance_chunk(test_case_voice_samples,actual_begin_voice_samples_index))
		for i in range (0,len(expected_utterance_chunk)-1):
			self.assertItemsEqual(expected_utterance_chunk[i],actual_utterance_chunk[i])
	def test_pre_process(self):
		expected_voice_samples = test_case_voice_samples
		actual_voice_samples = prep.pre_process(test_case_voice_samples)
		self.assertItemsEqual(expected_voice_samples,actual_voice_samples)
	def test_matrix_of_sample_numbers(self):
		expected_matrix_samples = np.array([5,6,9,18,20])
		actual_matrix_samples = prep.matrix_of_sample_numbers(test_RMS,test_case_voice_samples)
		self.assertItemsEqual(expected_matrix_samples,actual_matrix_samples)
	def test_selected_inflect_block_new(self):
		expected_selected_inflect_block = np.array([[4, 5, 6, 7, 8], [17, 18, 19, 20, 21, 22]],dtype=object)
		actual_selected_inflect_block = prep.selected_inflect_block_new(test_case_voice_samples_two)
		self.assertItemsEqual(expected_selected_inflect_block,actual_selected_inflect_block)
if __name__ == '__main__':
    unittest.main()

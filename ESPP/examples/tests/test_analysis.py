import unittest
from scipy.io import wavfile
import numpy as np
import pysptk
import os
from os.path import join
import sys
cwd = os.path.dirname(os.path.realpath(__file__))
_root = os.path.split(cwd)[0]
sys.path.append(_root)
import src.analysis as alysis


test_sampling_frequency = 44100
test_data = np.array([52,20,4,9,8,7,10,31,22,34,27,11,13,72,65])
test_chunk_size = 3

class TestAnalysis(unittest.TestCase):

	def test_root_mealysis_square(self):
		#expected_rms_value_pairs = [[52,20],[9,8],[10,31],[34,27],[13,72]]
		expected_rms_value = np.array([39.39,8.51,23.03,30.7])
		actual_rms_value = np.array(alysis.root_mean_square(test_data,test_chunk_size,test_sampling_frequency)) 
		for i in range(0,len(expected_rms_value)-1):
			self.assertAlmostEqual(expected_rms_value[i],actual_rms_value[i],places=1)

	def test_starting_info(self):
		expected_voiced_starting_info = [0.0 , 0.0 , 0.00013, 0.0002]
		actual_starting_info = alysis.starting_info(test_data,alysis.pitch_detect(test_data,test_sampling_frequency,test_chunk_size),test_sampling_frequency,test_chunk_size)
		for i in range(0,len(expected_voiced_starting_info)-1):
			self.assertAlmostEqual(expected_voiced_starting_info[i],actual_starting_info['voicedStart'][i],places=3)
	
	
	
if __name__ == '__main__':
    unittest.main()

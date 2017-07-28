import unittest
import numpy as np

test_case_utterance_time_stamps = np.arange(1,3,0.25)
# Normalize Function Module Testing from the synthesis module 
def normalize_function(test_case_utterance_time_stamps):
	normal = max(test_case_utterance_time_stamps) - min(test_case_utterance_time_stamps)
	test_case_utterance_time_stamps = (test_case_utterance_time_stamps-min(test_case_utterance_time_stamps))/normal
	normal_two = 0.5 - 0.01
	normailzed_utterance  = (test_case_utterance_time_stamps * normal_two) + 0.01
	return normailzed_utterance 

class TestSynthesis(unittest.TestCase):

	def test_normalize_function(self): 
		self.assertTrue(normalize_function(test_case_utterance_time_stamps)[-1] == 0.5)
		self.assertTrue(normalize_function(test_case_utterance_time_stamps)[0] ==  0.01)
	
	
if __name__ == '__main__':
    unittest.main()

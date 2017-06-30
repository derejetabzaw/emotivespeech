import unittest
from scipy.io import wavfile
import numpy
import pysptk
import os
from os.path import join
import sys
cwd = os.path.dirname(os.path.realpath(__file__))
_root = os.path.split(cwd)[0]
sys.path.append(_root)
import src.analysis as alysis
filename = "/home/dereje/Desktop/TestFolder/Test.wav"
fs,x = wavfile.read(filename)
Chunk_Size = 1024


class TestAnalysis(unittest.TestCase):
	def test_pitch_detect(self):
		self.assertIsNotNone(alysis.pitch_detect(x,fs, Chunk_Size))	
	def test_zero_crossing_rate_blocks(self):
		self.assertIsNotNone(alysis.zero_crossing_rate_blocks(x,Chunk_Size,fs))
	def test_root_mealysis_square(self):
		self.assertIsNotNone(alysis.root_mean_square(x,Chunk_Size,fs))
	def test_starting_info(self):
		self.assertIsNotNone(alysis.starting_info(x,alysis.pitch_detect(x,fs,Chunk_Size),fs,Chunk_Size))
	
	
if __name__ == '__main__':
    unittest.main()

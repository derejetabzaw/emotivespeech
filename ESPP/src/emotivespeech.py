import numpy as np
import preprocess as prep
import batchprocess as bp
import synthesis as syn
import os
import sys
import argparse
from optparse import OptionParser

def emotive_speech(
        fname,ofile,chunk_size,typeOfEmotion,
        semitones,gain,qfactor,speed,
        depth,tempo,intensity,parameter_control):
        
    """
    emotive_speech(fname, ofile, chunk_size, typeOfEmotion)
    A Caller Module
    Parameter:  fname
                ofile
                chunk_size
                typeOfEmotion
    Returns: output
    """
    fs, x = prep.wave_file_read(fname)
    time_stamps = bp.process_variables(x, fs, chunk_size)[0]
    consecutive_blocks = bp.process_variables(x, fs, chunk_size)[1]
    fundamental_frequency_in_blocks = bp.batch_analysis(x, fs, chunk_size)[0]
    voiced_samples = bp.batch_analysis(x, fs, chunk_size)[1]
    rms = bp.batch_analysis(x, fs, chunk_size)[2]
    selected_inflect_block = bp.batch_preprocess(
        fundamental_frequency_in_blocks, 
        voiced_samples, 
        rms)
    output = bp.batch_synthesis(
        fs,consecutive_blocks,time_stamps,selected_inflect_block,
        typeOfEmotion,semitones,cutfreq,gain,qfactor,speed,depth,
        tempo,intensity,parameter_control)
    output.build(fname, ofile)
    return output

if __name__ == '__main__':

    
    parser = argparse.ArgumentParser(description='Emotive Speech Generation', 
        formatter_class=argparse.RawTextHelpFormatter)
   
    parser.add_argument('f',
        metavar='filename',
        help = 'Absolute wavfile Directory')
    
    parser.add_argument('-c',
        metavar='chunk_size',
        type=int,
        default=1024,
        help='''Chunk Size: 
        Default Value=1024
        [...,256,512,1024,2048,4096...,]
        '''
        )
    parser.add_argument('typeOfEmotion',
        help='''happy,sad,afraid,happy_tensed
        '''
        )
    
    parser.add_argument('-s',
        metavar='semitones',
        type=float,
        default=1.5,
        help='''Semitones
        Default Values(H=1.5,HT=2.0,S=-1.5)
        '''
        )
    
    parser.add_argument('-r',
        metavar='cutfreq',
        type=int,
        default=3500,
        help='''Cut-Frequency
        ''')
   
    parser.add_argument('-g',
        metavar='gain',
        type=float,
        default=3.0,
        help='''Gain for Treble
        ''')
   
    parser.add_argument('-q',
        metavar='qfactor',
        type=int,
        default=1.0,
        help='''Q-factor
        ''')
  
    parser.add_argument('-v',
        metavar='speed',
        type=float,
        default=8.5,
        help='''Tremelo Speed
        Default Value(speed=8.5)
        WORKS ONLY FOR AFRAID!
        '''
        )
  
    parser.add_argument('-d',
        metavar='depth',
        type=float,
        default=60,
        help='''Tremelo Depth
        Default Value(depth=60)
        WORKS ONLY FOR AFRAID!
        ''')
   
    parser.add_argument('-o',
        metavar='tempo',
        type=float,
        default=1.0,
        help='''Tempo
        Default Values(S=0.95,A=1.05,H=1.1,HT=1.18
        '''
        )
   
    parser.add_argument('-i',
        metavar='intensity',
        type=float,
        default=3.0,
        help='''Gain for Intensity
        Default Value(3.0db)
        DOESN'T WORK FOR SAD PATCH!
        '''
        )
   
    parser.add_argument('-p',
        metavar='parameter_control',
        type=float,
        default=1.0,
        help='Parameter Control')
   
    args = parser.parse_args()
       
    fname = args.f
    chunk_size = args.c
    typeOfEmotion = args.typeOfEmotion
    ofile = '{}/out.wav'.format(os.path.dirname(fname))
    output_dir = '{}/{}'.format(os.path.dirname(fname), typeOfEmotion)
    ofile = '{}/{}'.format(output_dir, os.path.basename(fname))
    if not os.path.isdir(output_dir):
       os.makedirs(output_dir)
    semitones = args.s
    cutfreq = args.r
    gain = args.g
    qfactor = args.q
    speed = args.v
    depth = args.d
    tempo = args.o
    intensity = args.i
    parameter_control = args.p
    emotive_speech(
         fname,ofile,chunk_size,typeOfEmotion,
         semitones,gain,qfactor,speed,
         depth,tempo,intensity,parameter_control)


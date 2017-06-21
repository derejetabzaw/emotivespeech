#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import socket
import sys
sys.path.insert(0, '/home/dereje/Desktop/Git/emo_speech')
from src import EmotiveSpeech as es
from src import preprocess as pp
HOST = ''
PORT = 8000
ADDR = (HOST,PORT)
Chunk_Size = 1024
fs = 16000
serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind(ADDR)
serv.listen(5)

print 'listening ...'

while True:
  conn, addr = serv.accept()
  print 'client connected ... ', addr
  myfile = open('EmoTest.wav', 'w')

  while True:
    data = conn.recv(Chunk_Size)
    if not data: break
    
    myfile.write(data)
    print 'writing file ....'

  myfile.close()
  print 'finished writing file'
  GenFile = open('GenEmoTest.wav', 'w')
  fs,x = pp.wave_file_read('EmoTest.wav')
  output = es.emotive_speech(x,fs,"happy")
  GenFile.write(output)
  GenFile.close()
  conn.close()
print 'client disconnected'

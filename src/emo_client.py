#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys
import requests
import base64
import wave
import logging
import socket

HOST = 'localhost'
PORT = 8000
BUFSIZE = 1024
ADDR = (HOST,PORT)
wavefile = "/home/dereje/Desktop/tmp/Test1.wav"
#wavefile = "/home/hanson_dev/.hr/ttsserver/tmp"
bytes = open(wavefile).read()
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)
client.send(bytes)
client.close()

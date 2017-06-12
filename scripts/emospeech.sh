#!/usr/bin/env bash
# emospeech
#
# Usage: ./emospeech.sh 

echo "Executing the emospeech script..."
sleep 3
CWD=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
roslaunch emo_speech emospeech.launch

#!/usr/bin/env bash
#
# dev.sh
#
# This script will install dependecies for the emotive_sppech
echo "Library Dependecies..."
sleep 3
echo "Installing pysptk-Python Wrapper for Speech Processing Tool Kit"
sudo pip2 install pysptk
echo "Done Installing pysptk"
sleep 2
echo "Installing pysox-Python Wrapper for the SoX-Sound Exchange"
sudo pip2 install pysox
echo "Finished Installing pysox"

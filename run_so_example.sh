#! /bin/sh

DIR=$(pwd)
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$DIR"
echo $LD_LIBRARY_PATH
export LD_LIBRARY_PATH
python3 import_so_example.py

#!/usr/bin/env python

import sys
import subprocess
import niftk_file_utils as niftk
from random import sample

if len(sys.argv) < 7:
    print ('Usage: run_stereo_calibration_with_random_frames.py dirName numberOfPrefixCharacters leftEndsWith rightEndsWith numberOfSamples command')
    exit()

# Get all files in the specified path.
left_file_names =  niftk.get_files_by_ending(sys.argv[1], sys.argv[3])
right_file_names = niftk.get_files_by_ending(sys.argv[1], sys.argv[4])

# Get a random ordering of indexes, then try to find files where we have both left and right
file_indexes = sample(range(0, len(left_file_names)), len(left_file_names))
selected_left_file_names = []
selected_right_file_names = []
for i in file_indexes:
    if len(selected_left_file_names) < int(sys.argv[5]):
        left_name = left_file_names[file_indexes[i]]
        chopped_left_name = left_name[0:int(sys.argv[2])]
        matching = [m for m in right_file_names if chopped_left_name in m]
        if len(matching) == 1:
            selected_left_file_names.append(left_name)
            selected_right_file_names.append(matching[0])

# Sort out command
command = sys.argv
command.pop(0)
command.pop(0)
command.pop(0)
command.pop(0)
command.pop(0)
command.pop(0)
command.extend(selected_left_file_names)
command.extend(selected_right_file_names)

# Now run process.
return_code = subprocess.call(command)

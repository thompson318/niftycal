#!/usr/bin/env python

import os
import sys
import subprocess
from random import sample

if len(sys.argv) < 5:
    print 'Usage: launch_point_calibration_test.py dirName numberOfSamples endsWith [command]'
    exit()

def file_len(fname):
    p = subprocess.Popen(['wc', '-l', fname], stdout=subprocess.PIPE, 
                                              stderr=subprocess.PIPE)
    result, err = p.communicate()
    if p.returncode != 0:
        raise IOError(err)
    return int(result.strip().split()[0])

# Get all bmp files in the specified path.
fileNames = []
for f in os.listdir(sys.argv[1]):
    if f.endswith(sys.argv[3]) and file_len(f) > 50:
        fileNames.append(f)

# Get a sample of file indexes.
fileSamples = sample(range(0, len(fileNames)), int(sys.argv[2]))
fileSamples.sort()

# Now select the file names.
selectedFileNames = []
for f in fileSamples:
    selectedFileNames.append(fileNames[f])

# Sort out command
command = sys.argv
command.pop(0)
command.pop(0)
command.pop(0)
command.pop(0)
command.extend(selectedFileNames)

# Now run process.
returnCode = subprocess.call(command)

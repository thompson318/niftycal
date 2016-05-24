#!/usr/bin/env python

import os
import sys
from random import sample

if len(sys.argv) != 3:
    print 'Usage: launch_point_calibration_test.py dirName numberOfSamples'
    exit()

print 'Number of arguments:', len(sys.argv)
print 'Argument List:', str(sys.argv)
print 'Path to scan:', sys.argv[1]
print 'Number to pick:', sys.argv[2]

# Get all bmp files in the specified path.
fileNames = []
for f in os.listdir(sys.argv[1]):
    if f.endswith(".bmp"):
        fileNames.append(f)

# Get a sample of file indexes.
fileSamples = sample(range(0, len(fileNames)), int(sys.argv[2]))
fileSamples.sort()

# Now select the file names.
selectedFileNames = []
for f in fileSamples:
    selectedFileNames.append(fileNames[f])

# Now run calibration.
returnCode = subprocess.call()

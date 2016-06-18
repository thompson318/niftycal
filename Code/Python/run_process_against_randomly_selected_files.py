#!/usr/bin/env python

import sys
import subprocess
import niftk_file_utils as niftk
from random import sample

if len(sys.argv) < 6:
    print 'Usage: run_process_against_randomly_selected_files.py dirName endsWith minimumNumberLinesInFile numberOfSamples command'
    exit()

# Get all files in the specified path.
file_names = niftk.get_files_by_name_and_line_count(sys.argv[1], sys.argv[2], sys.argv[3])

# Get a sample of file indexes.
file_samples = sample(range(0, len(file_names)), int(sys.argv[4]))
file_samples.sort()

# Now select the file names.
selected_file_names = []
for f in file_samples:
    selected_file_names.append(file_names[f])

# Sort out command
command = sys.argv
command.pop(0)
command.pop(0)
command.pop(0)
command.pop(0)
command.pop(0)
command.extend(selected_file_names)

# Now run process.
return_code = subprocess.call(command)

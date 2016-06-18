#!/usr/bin/env python

import os
import subprocess

def number_of_lines(fname):
    p = subprocess.Popen(['wc', '-l', fname], stdout=subprocess.PIPE,
                                              stderr=subprocess.PIPE)
    result, err = p.communicate()
    if p.returncode != 0:
        raise IOError(err)
    return int(result.strip().split()[0])

def get_files_by_name_and_line_count(dirName, ending, lineCount):
    fileNames = []
    for f in os.listdir(dirName):
        if f.endswith(ending) and number_of_lines(f) >= int(lineCount):
            fileNames.append(f)
    return fileNames

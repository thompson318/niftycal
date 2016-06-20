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

def get_files_by_ending(dirName, ending):
    fileNames = []
    for f in os.listdir(dirName):
        if f.endswith(ending):
            fileNames.append(f)
    return fileNames

def get_files_by_ending_and_line_count(dirName, ending, lineCount):
    fileNames = get_files_by_ending(dirName, ending)
    longEnoughFiles = []
    for f in fileNames:
        if number_of_lines(f) >= int(lineCount):
            longEnoughFiles.append(f)
    return longEnoughFiles


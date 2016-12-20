#! /bin/bash

GRIDFILE=/home/thompson/build/NifTK-qt4.8.7/AprilTags/src/tags/tag16h5.ps
OUTDIR=/dev/shm/AprilGridTemp
mkdir -p $OUTDIR

#we'll use imagemagick's convert

convert $GRIDFILE $OUTDIR/tag.bmp

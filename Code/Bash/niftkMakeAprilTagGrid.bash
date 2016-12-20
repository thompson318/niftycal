#! /bin/bash
#we only need 24 tags for a chessboard equivalent grid, but we need more. if we're going to 
#extend the grid beyond the image
#GRIDFILE=/home/thompson/build/NifTK-qt4.8.7/AprilTags/src/tags/tag25h7.ps
GRIDFILE=/home/thompson/work/build/NifTK-dbg/AprilTags/src/tags/tag25h7.ps
OUTDIR=/dev/shm/AprilGridTemp
mkdir -p $OUTDIR

#we'll use imagemagick's convert

#convert $GRIDFILE $OUTDIR/tag.bmp

files=$(ls $OUTDIR)

for file in $files 
do
	echo convert $file -crop 612x612+0+90 ${file#.*}_cropped.bmp
done



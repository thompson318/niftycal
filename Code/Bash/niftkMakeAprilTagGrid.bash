#! /bin/bash
#we only need 24 tags for a chessboard equivalent grid, but we need more. if we're going to 
#extend the grid beyond the image
GRIDFILE=/home/thompson/build/NifTK-qt4.8.7/AprilTags/src/tags/tag25h7.ps
#GRIDFILE=/home/thompson/work/build/NifTK-dbg/AprilTags/src/tags/tag25h7.ps
OUTDIR=/dev/shm/AprilGridTemp
mkdir -p $OUTDIR

#we'll use imagemagick's convert

if [ $GRIDFILE -nt $OUTDIR/000.bmp ]
then
	convert $GRIDFILE $OUTDIR/%03d.bmp
fi

files=$(ls $OUTDIR/???.bmp)

for file in $files 
do
	if [ $OUTDIR/${file} -nt ${OUTDIR}/${file%.*}_cropped.bmp ]
	then
		convert ${OUTDIR}/$file -crop 612x612+0+90 ${OUTDIR}/${file%.*}_cropped.bmp
	fi
done

#6x4 grid.
files24=$(ls ${OUTDIR}/???_cropped.bmp | head -n 24)

echo $files24

montage $files24 -geometry 612x612+0+0 -tile 6x4 montage24.bmp

#make it bigger, 18 x 12 = 216

files216=$(ls ${OUTDIR}/???_cropped.bmp | head -n 216)

montage $files216 -geometry 612x612+0+0 -tile 18x12 montage216.bmp

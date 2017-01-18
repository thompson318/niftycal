#! /bin/bash
APRILTAG_SRC=$1
GRIDFILE=${APRILTAG_SRC}/tags/tag25h7.ps
OUTDIR=./AprilGridsTemp
mkdir -p $OUTDIR

#check we have imagemagick for convert and montage

if [ $# -ne 1 ]
then
	echo "usage " $0 " pathToAprilTagSource"
	echo "reads tag25h7.ps and creates a 6x4 and 18x12 grid of AprilTags"
	exit
fi


if [ ! $(which convert 2> /dev/null) ]
then
	echo "ImageMagick not found on your path, please check installation"
	exit
fi

if [ ! -e $GRIDFILE ]
then
	echo "Failed to find " $GRIDFILE
	exit
fi

if [ ! -e $OUTDIR/000.bmp ] || [ $GRIDFILE -nt $OUTDIR/000.bmp ]
then
	convert $GRIDFILE $OUTDIR/%03d.bmp
fi

files=$(ls $OUTDIR/???.bmp)

for file in $files
do
	if [ ${file} -nt ${file%.*}_cropped.bmp ]
	then
		convert $file -crop 612x612+0+90 ${file%.*}_cropped.bmp
	fi
done

#6x4 grid.

if [ ${OUTDIR}/000_cropped.bmp -nt montage24.bmp ]
then

	#we only need 24 tags for a chessboard equivalent grid, but we need more. if we're going to
	#extend the grid beyond the image
	files24=$(ls ${OUTDIR}/???_cropped.bmp | head -n 24)

	montage $files24 -geometry 612x612+0+23 -tile 6x4 montage24.bmp

	#make it bigger, 18 x 12 = 216

	files216=$(ls ${OUTDIR}/???_cropped.bmp | head -n 216)

	montage $files216 -geometry 612x612+0+23 -tile 18x12 montage216.bmp
fi

#otherwise lets just work out image sizes.
#each square is 612x612 and the tag is starts at pixel 68.
#so tag size is 612 - (68 * 2) = 476 pixels
#tag corner to corner width = 612 * 6 - 68-2 = 3536 pixels. We want this equal 39 mm, same as chessboard,
#pixTOmm=39  / 3536
#image width in mm = 612*6 * 39/3536 = 40.50 mm
#need to fiddle the montage setting to get 2 more mm into vertical size
#1 * 3536 / 39 = 90 pixels
#136/4 = 34 pixel per patch

#13.02 percent scaling



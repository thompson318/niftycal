#! /bin/bash

if [ ! $(which convert 2> /dev/null) ]
then
	echo "ImageMagick not found on your path, please check installation"
	exit
fi

COLUMNS=14
ROWS=10

SPACING=3.0
OUTERDIAMETER=2.0
INNERDIAMETER=1.0
BORDER=3.0

PIXELSPERMM=240

SPACING_PIX=$(echo ${SPACING}*${PIXELSPERMM} | bc -l)
OUTERDIAMETER_PIX=$(echo ${OUTERDIAMETER}*${PIXELSPERMM} | bc -l)
INNERDIAMETER_PIX=$(echo ${INNERDIAMETER}*${PIXELSPERMM} | bc -l)
BORDER_PIX=$(echo $BORDER*${PIXELSPERMM} | bc -l)



#make the template image
INNERCOLS=0
INNERROWS=0
IMAGEWIDTH_PIX=$(echo ${INNERCOLS}*${SPACING_PIX}+${BORDER_PIX} | bc -l)
IMAGEHEIGHT_PIX=$(echo ${INNERROWS}*${SPACING_PIX}+${BORDER_PIX} | bc -l)
drawString=""
row=0
column=0

xcentre=$(echo ${SPACING_PIX}*${column}+${BORDER_PIX}/2 | bc )
xedge=$(echo ${xcentre}+${OUTERDIAMETER_PIX}/2 | bc )
xinneredge=$(echo ${xcentre}+${INNERDIAMETER_PIX}/2 | bc )

ycentre=$(echo ${SPACING_PIX}*${row}+${BORDER_PIX}/2 | bc )

drawString=$(echo ${drawString} fill black circle $xcentre,$ycentre $xedge,$ycentre fill white circle $xcentre,$ycentre $xinneredge,$ycentre)
	
convert -size ${IMAGEWIDTH_PIX}x${IMAGEHEIGHT_PIX} xc:white \
	-depth 8 \
	-draw "${drawString}" \
	-blur 40x100 ringsTemplateImage.bmp

#make the reference image and point positions, we need a bigger border so we don't fall off the edge

BORDER=12.0
BORDER_PIX=$(echo $BORDER*${PIXELSPERMM} | bc -l)
INNERCOLS=$(($COLUMNS-1))
INNERROWS=$(($ROWS-1))
IMAGEWIDTH_PIX=$(echo ${INNERCOLS}*${SPACING_PIX}+${BORDER_PIX} | bc -l)
IMAGEHEIGHT_PIX=$(echo ${INNERROWS}*${SPACING_PIX}+${BORDER_PIX} | bc -l)

drawString=""
column=0
rm ringsReferencePoints.txt
pointIndex=0
while [ $column -lt $COLUMNS ]
do
	row=0
	while [ $row -lt $ROWS ]
	do	
		xcentre=$(echo ${SPACING_PIX}*${column}+${BORDER_PIX}/2 | bc )
		xedge=$(echo ${xcentre}+${OUTERDIAMETER_PIX}/2 | bc )
		xinneredge=$(echo ${xcentre}+${INNERDIAMETER_PIX}/2 | bc )

		ycentre=$(echo ${SPACING_PIX}*${row}+${BORDER_PIX}/2 | bc )
		
		echo  $pointIndex $xcentre $ycentre >> ringsReferencePoints.txt
		pointIndex=$(($pointIndex + 1 ))
		drawString=$(echo ${drawString} fill black circle $xcentre,$ycentre $xedge,$ycentre fill white circle $xcentre,$ycentre $xinneredge,$ycentre)
		row=$(($row+1))
	done
	column=$(($column+1))
done

convert -size ${IMAGEWIDTH_PIX}x${IMAGEHEIGHT_PIX} xc:white \
	-depth 8 \
	-draw "${drawString}" \
	-blur 40x100 ringsReferenceImage.bmp

	

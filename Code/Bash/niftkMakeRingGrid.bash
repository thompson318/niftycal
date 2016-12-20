#! /bin/bash


COLUMNS=14
ROWS=10

SPACING=3.0
OUTERDIAMETER=2.0
INNERDIAMETER=1.0
BORDER=3.0

PIXELSPERMM=12 


SPACING_PIX=$(echo ${SPACING}*${PIXELSPERMM} | bc -l)
OUTERDIAMETER_PIX=$(echo ${OUTERDIAMETER}*${PIXELSPERMM} | bc -l)
INNERDIAMETER_PIX=$(echo ${INNERDIAMETER}*${PIXELSPERMM} | bc -l)
BORDER_PIX=$(echo $BORDER*${PIXELSPERMM} | bc -l)

IMAGEWIDTH_PIX=$(echo ${COLUMNS}*${SPACING_PIX}+${BORDER_PIX}*2 | bc -l)
IMAGEHEIGHT_PIX=$(echo ${ROWS}*${SPACING_PIX}+${BORDER_PIX}*2 | bc -l)

drawString=""
row=0
while [ $row -lt $ROWS ]
do
	column=0
	while [ $column -lt $COLUMNS ]
	do	
		xcentre=$(echo ${OUTERDIAMETER_PIX}/2+${SPACING_PIX}*${column}+${BORDER_PIX} | bc )
		xedge=$(echo ${xcentre}+${OUTERDIAMETER_PIX}/2 | bc )
		xinneredge=$(echo ${xcentre}+${INNERDIAMETER_PIX}/2 | bc )

		ycentre=$(echo ${OUTERDIAMETER_PIX}/2+${SPACING_PIX}*${row}+${BORDER_PIX} | bc )

		drawString=$(echo ${drawString} fill black circle $xcentre,$ycentre $xedge,$ycentre fill white circle $xcentre,$ycentre $xinneredge,$ycentre)
		column=$(($column+1))
	done
	row=$(($row+1))
done

convert -size ${IMAGEWIDTH_PIX}x${IMAGEHEIGHT_PIX} xc:white  \
	-draw "${drawString}" rings.bmp
	

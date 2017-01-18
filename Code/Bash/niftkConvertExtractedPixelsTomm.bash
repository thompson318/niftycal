#! /bin/bash

#I used extract corners on the bitmap images, now convert to mm a 300 ppi

input=$1

originx=$(head -n 1 $input | cut -d " " -f 2)
originy=$(head -n 1 $input | cut -d " " -f 3)

toInch=$(echo 1/300 | bc -l)
toMM=$(echo 25.4/300 | bc -l)

i=1
lines=$(grep -c $ $input)

while [ $i -le $lines ] 
do
	
	name=$(head -n $i $input | tail -n 1 | cut -d " " -f 1)
	x=$(head -n $i $input | tail -n 1 | cut -d " " -f 2)
	y=$(head -n $i $input | tail -n 1 | cut -d " " -f 3)

	xmm=$(echo "($x - $originx) * $toMM" | bc -l) 
	ymm=$(echo "($y - $originy) * $toMM" | bc -l) 

	echo $name $xmm $ymm 0
	i=$(($i+1))
done


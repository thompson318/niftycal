#!/usr/bin/env python

import sys

if len(sys.argv) != 1:
    print ('Usage: make_rings_grid')
    exit()

number_of_grids = 2
number_of_columns = 15
number_of_rows = 7
spacing = 5.0
outer_diameter = 2.0
inner_diameter = 1.0
border = 15.0
pixels_per_millimetre = 12

spacing_in_pixels        = spacing        * pixels_per_millimetre
outer_diameter_in_pixels = outer_diameter * pixels_per_millimetre
inner_diameter_in_pixels = inner_diameter * pixels_per_millimetre
border_in_pixels         = border         * pixels_per_millimetre

inner_cols = number_of_columns - 1
inner_rows = number_of_rows - 1

image_width_in_pixels = inner_cols * (spacing_in_pixels/2) + border_in_pixels
image_height_in_pixels = inner_rows * spacing_in_pixels + border_in_pixels

draw_string = str("")

counter = 0

for g in range(0, number_of_grids):
    for c in range(0, number_of_columns):
        for r in range(0, number_of_rows):
            xcentre    = (g * image_width_in_pixels) + (spacing_in_pixels * c / 2) + border_in_pixels / 2.0
            xedge      = xcentre + outer_diameter_in_pixels / 2
            xinneredge = xcentre + inner_diameter_in_pixels / 2.0

            if c % 2 == 1:
                ycentre = spacing_in_pixels * r + border_in_pixels / 2.0 + spacing_in_pixels / 2.0
            else:
                ycentre = spacing_in_pixels * r + border_in_pixels / 2.0

            draw_string = draw_string + " fill black circle " + str(xcentre) + "," + str(ycentre) + " " + str(xedge) + "," + str(ycentre)
            draw_string = draw_string + " fill white circle " + str(xcentre) + "," + str(ycentre) + " " + str(xinneredge) + "," + str(ycentre)

            print (str(counter) + " " + str(xcentre) + " " + str(ycentre))
            counter += 1

command_string = str("convert -size ") + str(int(number_of_grids * image_width_in_pixels)) + "x" + str(int(image_height_in_pixels)) + " xc:white -draw \"" + draw_string + "\" rings.bmp"
print (command_string)


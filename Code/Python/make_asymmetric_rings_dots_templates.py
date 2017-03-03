#!/usr/bin/env python

import sys
import subprocess

if len(sys.argv) != 1:
    print ('Usage: make_asymmetric_rings_grid')
    exit()

def make_asymmetric_rings_grid(ngrids, ncols, nrows, spacing_mm, outer_dia_mm, inner_dia_mm, border_mm, pixels_per_mm, do_rings, output_prefix):

    spacing_in_pixels = spacing_mm * pixels_per_mm
    outer_diameter_in_pixels = outer_dia_mm * pixels_per_mm
    inner_diameter_in_pixels = inner_dia_mm * pixels_per_mm
    border_in_pixels = border_mm * pixels_per_mm

    inner_cols = ncols - 1
    inner_rows = nrows - 1

    image_width_in_pixels = inner_cols * (spacing_in_pixels/2) + border_in_pixels
    image_height_in_pixels = inner_rows * spacing_in_pixels + border_in_pixels

    draw_string = str("")

    counter = 0

    for g in range(0, ngrids):
        for c in range(0, ncols):
            for r in range(0, nrows):
                xcentre = (g * image_width_in_pixels) + (spacing_in_pixels * c / 2) + border_in_pixels / 2.0
                xedge = xcentre + outer_diameter_in_pixels / 2
                xinneredge = xcentre + inner_diameter_in_pixels / 2.0

                if c % 2 == 1:
                    ycentre = spacing_in_pixels * r + border_in_pixels / 2.0 + spacing_in_pixels / 2.0
                else:
                    ycentre = spacing_in_pixels * r + border_in_pixels / 2.0

                draw_string = draw_string + " fill black circle " + str(xcentre) \
                    + "," + str(ycentre) + " " + str(xedge) + "," + str(ycentre)

                if do_rings:
                    draw_string = draw_string + " fill white circle " + str(xcentre) \
                        + "," + str(ycentre) + " " + str(xinneredge) + "," + str(ycentre)

                print (str(counter) + " " + str(xcentre) + " " + str(ycentre))
                counter += 1

    image_file_name = str(output_prefix) + "-"\
                      + str(ngrids) + "x" \
                      + str(ncols) + "x" \
                      + str(nrows) + "x" \
                      + str(pixels_per_mm) \
                      + ".bmp"

    command_string = str("convert -size ") + str(int(number_of_grids * image_width_in_pixels)) \
        + "x" + str(int(image_height_in_pixels)) + " xc:white -draw \"" + draw_string + "\" " + image_file_name

    print (command_string)


def make_template(ngrids, ncols, nrows, outer_dia_mm, inner_dia_mm, border_mm, pixels_per_mm, do_rings, output_prefix):
    border_in_pixels = border_mm * pixels_per_mm
    outer_diameter_in_pixels = outer_dia_mm * pixels_per_mm
    inner_diameter_in_pixels = inner_dia_mm * pixels_per_mm
    image_width_in_pixels = border_in_pixels
    image_height_in_pixels = image_width_in_pixels

    xcentre = border_in_pixels / 2
    xedge = xcentre + (outer_diameter_in_pixels / 2)
    xinneredge = xcentre + (inner_diameter_in_pixels / 2)

    ycentre = xcentre

    draw_string = "" + " fill black circle " + str(xcentre) \
        + "," + str(ycentre) + " " + str(xedge) + "," + str(ycentre)

    if do_rings:
        draw_string = draw_string + " fill white circle " + str(xcentre) + "," \
            + str(ycentre) + " " + str(xinneredge) + "," + str(ycentre)

    image_file_name = str(output_prefix) + "-" \
                      + str(ngrids) + "x" \
                      + str(ncols) + "x" \
                      + str(nrows) + "x" \
                      + str(pixels_per_mm) \
                      + "-template.bmp"

    command_string = str("convert -size ") + str(int(image_width_in_pixels)) \
        + "x" + str(int(image_height_in_pixels)) + " xc:white -depth 8 -draw \"" + draw_string + "\" "\
        + " -blur 40x100 " + image_file_name

    print (command_string)


# Main start of program

# Note: This is clearly not finished, as we can't parse command line args and so on.
#       I also had a problem with calling such a long command_string.
#       So at the moment we just write to console, and you can paste the command yourself.


# Non-coplanar, dots, to print at 12pix per millimetre.
number_of_grids = 2
number_of_columns = 15
number_of_rows = 7
spacing = 5
outer_diameter = 2
inner_diameter = 1
border = 15
pixelmm = 12
rings = False
prefix = "dots-noncoplanar"

make_asymmetric_rings_grid(number_of_grids, number_of_columns, number_of_rows, spacing, \
                           outer_diameter, inner_diameter, border, pixelmm, rings, prefix)


# Non-coplanar, dots, used as a reference for template matching.
pixelmm = 120
prefix = "dots-noncoplanar-reference"

make_asymmetric_rings_grid(number_of_grids, number_of_columns, number_of_rows, spacing, \
                           outer_diameter, inner_diameter, border, pixelmm, rings, prefix)

# Non-coplanar, dots, used as a template for template matching.
border = 3
prefix = "dots-noncoplanar"
make_template(number_of_grids, number_of_columns, number_of_rows, outer_diameter, inner_diameter, \
              border, pixelmm, rings, prefix)


# Coplanar, dots, to print at 12pix per millimetre.
number_of_grids = 1
number_of_columns = 23
number_of_rows = 7
pixelmm = 12
border = 15
prefix = "dots-coplanar"

make_asymmetric_rings_grid(number_of_grids, number_of_columns, number_of_rows, spacing, \
                           outer_diameter, inner_diameter, border, pixelmm, rings, prefix)


# Coplanar, dots, used as a reference for template matching.
pixelmm = 120
prefix = "dots-coplanar-reference"

make_asymmetric_rings_grid(number_of_grids, number_of_columns, number_of_rows, spacing, \
                           outer_diameter, inner_diameter, border, pixelmm, rings, prefix)

# Coplanar, dots, used as a template for template matching.
border = 3
prefix = "dots-coplanar"
make_template(number_of_grids, number_of_columns, number_of_rows, outer_diameter, inner_diameter, \
              border, pixelmm, rings, prefix)
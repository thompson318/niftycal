#! /usr/bin/python

import matplotlib.pyplot as plt

columns=14
rows=10

ring_outer_diameter=3.0
ring_inner_diameter=1.5
ring_spacing=3.0

x=0
y=0

fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
for x in range (columns):
    for y in range (rows):
        print ( x*ring_spacing, " : " , y*ring_spacing)
        outer = plt.Circle((x*ring_spacing, y*ring_spacing), ring_outer_diameter, color='black')
        inner = plt.Circle((x*ring_spacing, y*ring_spacing), ring_inner_diameter, color='w')

        ax.add_artist(outer)
        ax.add_artist(inner)

# (or if you have an existing figure)
# fig = plt.gcf()
# ax = fig.gca()


plt.show(fig)
#fig.savefig('plotcircles.png')

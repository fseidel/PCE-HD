#!/usr/bin/env python

from PIL import Image
import sys
import os
import numpy as np
import itertools

argv = sys.argv
argc = len(argv)

if argc < 2:
    print "Please provide input file"
    sys.exit(0)

ifname = sys.argv[1]
ofname = os.path.splitext(ifname)[0]+".txt"

with Image.open(ifname) as f:
    img = np.array(f)
    
width = len(img[0])
print width
height = len(img)
print height

towrite = ''
for i, row in enumerate(img):
    for j, col in enumerate(row):
        for color in col:
            towrite += str(color) + ' '
        if(j == width-1):
            towrite += '1 ' #hsync
            if(i == height-1):
                towrite += '1'
            else: towrite += '0'
        else: towrite += '0 0'
        towrite += '\n'

with open(ofname, 'w') as f:
    f.write(towrite)
    
    
"""
for row in img:
    for col in row:
        print col
""" 

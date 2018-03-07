#!/usr/bin/env python

scalefactor = 1

import sys
import os
import pygame
import gzip


res  = (525+8*5, 485)

def plot_pixel(screen, where, color):
    row = where[0]*scalefactor
    col = where[1]*scalefactor
    if col >= res[0]: return
    for i in xrange(0, scalefactor):
        for j in xrange(0, scalefactor):
            screen.set_at((row+i, col+j), color)
    return


done = False

argv = sys.argv
argc = len(argv)

if argc < 2:
    print "Please provide input file"
    sys.exit(0)

filename = sys.argv[1]
ext = os.path.splitext(filename)[1]

if(ext == '.gz'):
    with gzip.open(filename) as f:
        data = f.read()
else:
    with open(filename) as f:
        data = f.read()
        
data = data.split('\n')

pygame.init()

size = (res[0]*scalefactor, res[1]*scalefactor)

screen = pygame.display.set_mode(size)
pygame.display.set_caption("TVEmu")

row = 0
col = 0

prev_H = 1
prev_V = 1

#scanline = pygame.Rect(0, 0, 640, scalefactor)


#H and V are ACTIVE LOW!

for line in data:
    if not line or line[0] == '#': continue
    (R, G, B, H, V) = map(int, line.split())
    #print (R, G, B, H, V)
    col += 1
    do_HSYNC = H and not prev_H #start on rising edge of sync line
    do_VSYNC = V and not prev_V #ditto
    if do_HSYNC:
        print "line" + str(row) + "done"
        col = 0
        row += 2
        """
        print row
        pygame.display.update(scanline)
        scanline.move_ip(0, scalefactor)
        wait = True
        event = pygame.event.get()
        while wait:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    wait = False
                    break
        """
    if do_VSYNC:
        pygame.display.flip()
        row = 0
        nextframe = 0
        print "Vsync detected. Press any key to render next frame"
        while not nextframe:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    nextframe = 1
    if(V and H): plot_pixel(screen, (col, row), (R, G, B))
    prev_H = H
    prev_V = V

pygame.display.flip()
print "Done"

while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

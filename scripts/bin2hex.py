#!/usr/bin/python
#thanks to https://stackoverflow.com/a/1035360

import sys
import os

wordsize = int(sys.argv[1])
filename = sys.argv[2]

fmt = '0' + str(2*wordsize) + 'x'
with open(filename, "rb") as infile:
    arr = bytearray(infile.read())
    
noext = os.path.splitext(filename)[0]

with open(noext + '.hex', "wb") as outfile:
    i = 0
    while(i < len(arr)):
        word = 0
        for x in range(0, wordsize):
            word |= arr[i+x] << x
        i += wordsize
        outfile.write(format(word, fmt) + '\n')
            

#!/usr/bin/python
#thanks to https://stackoverflow.com/a/1035360

import sys
import os

filename = sys.argv[1]
with open(filename, "rb") as infile:
    noext = os.path.splitext(filename)[0]
    with open(noext + '.hex', "wb") as outfile:
        byte = infile.read(1)
        while byte:
            outfile.write(format(ord(byte), '02x') + '\n')
            byte = infile.read(1)
            

#!/bin/sh
wla-huc6280 test.S
wlalink linkfile test.bin
../../scripts/bin2hex.py 1 test.bin


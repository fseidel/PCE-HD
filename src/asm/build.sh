#!/bin/sh
wla-huc6280 test.S
wlalink linkfile test.bin
./bin2hex.py test.bin


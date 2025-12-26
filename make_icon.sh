#!/bin/bash
set -e
magick convert data/icon.png -define icon:auto-resize:256,128,96,64,48,32,16 -compress zip data/icon.ico
#! /bin/sh
mkdir -p ${1}/output
mogrify -density 1200 -quality 100 -path "${1}/output/" -format png -background white -alpha remove -alpha off "${1}/*.pdf" 
ffmpeg -framerate 10 -pattern_type glob -i "${1}/output/Wake_*.png" Wake.mp4
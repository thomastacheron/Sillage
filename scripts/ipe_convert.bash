#! /bin/sh
mkdir -p ${1}/png

for f in ${1}/*; do
    # echo -e ${f%.*} ${f##*.} ${f##*/}
    filename=${f##*/}
    fileradical=${filename%.*}
    echo ${filename} ${fileradical}
    if [ ${fileradical} != "png" ]
    then
        iperender -png -resolution 1000 ${1}/${filename} ${1}/png/${fileradical}.png
    fi
done

ffmpeg -framerate 10 -pattern_type glob -i "${1}/png/*.png" ${2}.mp4
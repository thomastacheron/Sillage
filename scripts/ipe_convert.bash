#! /bin/sh
mkdir -p ${1}/png

for f in ${1}/*; do
    # echo -e ${f%.*} ${f##*.} ${f##*/}
    filename=${f##*/}
    fileradical=${filename%.*}
    if [ ${fileradical} != "png" ]
    then
        if [ ! -f ${1}/png/${fileradical}.png ]
        then
            echo ${fileradical}
            iperender -png -resolution 300 ${1}/${filename} ${1}/png/${fileradical}.png
        fi
    fi
done

ffmpeg -framerate 20 -pattern_type glob -i "${1}/png/*.png" ${2}.mp4

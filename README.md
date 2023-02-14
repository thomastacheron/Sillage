# Sillage
Boat localisation using wake and interval analysis. Last year application project at Guerlédan Lake.

This is a test of https://github.com/Teusner/WakeBoat project for boat localisation, using true detections with two buoys.

![out](https://user-images.githubusercontent.com/114411509/218680032-6cc96afe-0677-40e0-886c-70fd600703ae.gif)

This repository implement the boat localisation given the wakes detected. To detect the wakes see [this repository](https://github.com/Pazu35/sillage).
# User manual

Replace ```<YOUR_PATH>``` with the path of your choice e.g. ```~/Documents```.

## Dependencies

### Libraries
```bash
sudo apt install libipe-dev libpng-dev libjpeg-dev libspiro-dev
```

### ipe_generator
```bash
cd <YOUR_PATH>
git clone https://github.com/thomastacheron/ipe_generator.git
cd ipe_generator
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

### fmt
```bash
cd <YOUR_PATH>
git clone https://github.com/fmtlib/fmt.git
cd fmt
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

## Install

```bash
cd <YOUR_PATH>
git clone https://github.com/thomastacheron/Sillage.git
cd Sillage
mkdir build
cd build
cmake ..
make -j8
```

## Setup

```bash
cd <YOUR_PATH>/Sillage
git submodule init
git submodule update
```

## Generate .ipe files
```bash
cd <YOUR_PATH>/Sillage
mkdir output
cd build
cmake ..
make -j8
./main --path=../output
```

## Generate .png images
```bash
cd <YOUR_PATH>/Sillage
cd scripts
./ipe_convert.bash ../output
```

## Generate .mp4 video
```bash
cd <YOUR_PATH>/Sillage
cd scripts
./ipe_convert.bash ../output
./video.bash ../output
```

You will get ```<YOUR_PATH/Sillage/scripts/Wake.mp4```

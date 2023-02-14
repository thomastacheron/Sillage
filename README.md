# Sillage
Boat localisation using wake and interval analysis. Last year application project at Guerlédan Lake.

This is a test of https://github.com/Teusner/WakeBoat project for boat localisation, using true detections with two buoys.

![out](https://user-images.githubusercontent.com/114411509/218680032-6cc96afe-0677-40e0-886c-70fd600703ae.gif)

# Dependencies

```bash
git clone https://github.com/ThomasLeMezo/ipe_generator.git
cd ipe_generator
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

# Setup

```bash
cd <YOUR_PATH>/Sillage
git submodule init
git submodule update
```

# Generate .ipe files
```bash
cd <YOUR_PATH>/Sillage
mkdir output
cd build
cmake ..
make -j8
./main --path=../output
```

# Generate .png images
```bash
cd <YOUR_PATH>/Sillage
cd scripts
./ipe_convert.bash ../output
```

# Generate .mp4 video
```bash
cd <YOUR_PATH>/Sillage
cd scripts
./ipe_convert.bash ../output
./video.bash ../output
```

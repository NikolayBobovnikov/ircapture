AuViR Solutions:

--- auvir_model: reconstructs 3D models of scenes and objects
--- auvir_embed: 
    --- auvir_suite: wearing IR marker suite which provides accel measurements via wifi
    --- auvir_picam: wifi Raspberry Pi camera with realtime onboard image preprocessing (thresholding and packing)
--- auvir_motion (depends on: auvir_suite): real time motion capture (for omnimotion controller)
--- auvir_openscene (depends_on: auvir: picam): calculate player's positions in realtime based on collected data from PIDetectors around the scene
--- auvir_game: self-explanatory
--- auvir_website: self-explanatory



===== Set upp dev environment log

0. sudo ./install_all
1. install newer cmake 3.2 instead of 2.8 (needless?). For this, aptitude remove {cmake,qt-sdk}
2. gtest : download, make and instal

cd gtest-1.7.0
./configure
make
sudo cp -a include/gtest /usr/include
sudo cp -a lib/.libs/* /usr/lib
sudo ldconfig -v | grep gtest

if output contains libgtest.so, everything is OK

3. 


ProjectFolder=$(pwd)
LibraryFolder="$ProjectFolder/Libraries"
YDLidarFolder="$LibraryFolder/YDLidar"

pip install cmake
pip install swig

cd "$YDLidarFolder"
pip install .

pip install pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod 

pip install netifaces
pip install pyPS4Controller
pip install tflite-runtime
pip3 install picamera2
pip install opencv-python
pip install matplotlib
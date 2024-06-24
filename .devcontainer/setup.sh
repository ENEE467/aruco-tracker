# !/bin/bash

set -e

PROJECT_ROOT=$HOME/line-follower-tracker

if [ ! -d $PROJECT_ROOT ]
then
  printf "Setup failed: Project root directory not found\n"
fi
sudo usermod -aG video 467-dev

# Install some dependencies for OpenCV and its modules
sudo apt update && sudo apt install -y \
  python2 \
  default-jdk \
  default-jre \
  ffmpeg \
  qtbase5-dev \
  qt5-qmake \
  libglu1-mesa-dev \
  mesa-common-dev \
  freeglut3-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-gl \
  gstreamer1.0-gtk3 \
  gstreamer1.0-qt5 \
  gstreamer1.0-pulseaudio \
  libjpeg-dev \
  libtiff-dev \
  libopenjp2-7 \
  libopenjp2-7-dev \
  libopenjp2-tools \
  libopenjp3d-tools \
  libopenjp3d7 \
  libopenjpip-dec-server \
  libopenjpip-server \
  libopenjpip-viewer \
  libopenjpip7 \
  openjpeg-doc

pip3 install pylint flake8 vtk

# Clone OpenCV Project ------------------------------------------------------
if [ ! -d opencv-* ]
then
    wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.9.0.zip
    unzip opencv.zip && rm opencv.zip
fi

# Clone the modules ------------------------------------------------------
if [ ! -d opencv_contrib ]
then
    git clone -b 4.x https://github.com/opencv/opencv_contrib.git
fi

# Build the library with the aruco module and install it
if [ -d opencv-4.9.0/build ]
then
  rm -rf opencv-4.9.0/build
fi

mkdir -p opencv-4.9.0/build && cd opencv-4.9.0/build
cmake -D OPENCV_EXTRA_MODULES_PATH=~/line-follower-tracker/opencv_contrib/modules \
      -D BUILD_LIST=calib3d,highgui,objdetect,aruco,videoio \
      -D WITH_OPENGL=ON \
      -D WITH_QT=ON \
      ..

make -j$(nproc)
sudo make install

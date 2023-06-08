#!/bin/sh

basedir=`pwd`

# Build the gpiod library
sudo apt-get install -y autoconf-archive
git clone https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod
cd libgpiod
git checkout v1.6.3
./autogen.sh --enable-tools=yes --prefix=/usr
make
sudo make install
sudo ldconfig

# build the gpioctrl service
cd $basedir
mkdir -p build && cd build
cmake ..
make
sudo make install
cd ..
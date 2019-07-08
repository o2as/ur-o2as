#!/usr/bin/env bash

 apt-get -y update \
    &&  apt-get -y upgrade \
    &&  apt-get install -y vim screen tree ssh

 apt -y install software-properties-common \
	&&  add-apt-repository -y ppa:freecad-maintainers/freecad-stable \
	&&  add-apt-repository -y ppa:ubuntu-toolchain-r/test \
	&&  add-apt-repository -y ppa:git-core/ppa \
	&&  apt update \
	&&  apt -y upgrade \
	&&  apt -y install cmake cmake-curses-gui cmake-qt-gui libgtk2.0-dev pkg-config doxygen graphviz chrpath \
	libavcodec-dev libavformat-dev libswscale-dev libxt-dev python-numpy \
	libeigen3-dev libflann-dev libusb-1.0-0-dev libqhull-dev libtiff5-dev libavahi-client-dev \
	libbz2-dev makeself mesa-common-dev python2.7-dev gawk dh-autoreconf \
	liboce-foundation-dev liboce-modeling-dev liboce-ocaf-dev libxerces-c-dev

 add-apt-repository -y ppa:webupd8team/sublime-text-3 \
	&&  apt update \
	&&  apt -y upgrade \
	&&  apt -y install meshlab sublime-text-installer freecad searchmonkey \
	&&  apt -y install libxmlrpc-c++8 libxmlrpc-c++8-dev

wget https://github.com/NixOS/patchelf/archive/0.9.tar.gz
tar -xzvf 0.9.tar.gz
cd patchelf-0.9
./bootstrap.sh
./configure
 make install

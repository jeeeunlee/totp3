#! /bin/bash

echo "install eigenpy"
echo "https://github.com/stack-of-tasks/eigenpy"

if [ -d ~/dependency_ws ];
then
    cd ~/dependency_ws
else
    cd ~/ && mkdir dependency_ws && cd dependency_ws
fi

git clone --recursive https://github.com/stack-of-tasks/eigenpy &&
cd eigenpy && git checkout master &&
mkdir build && cd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 &&
make -j4 &&
sudo make install
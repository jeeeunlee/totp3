#! /bin/bash


echo "install pinocchio"
echo "https://stack-of-tasks.github.io/pinocchio/download.html"

if [ -d ~/dependency_ws ];
then
    cd ~/dependency_ws
else
    cd ~/ && mkdir dependency_ws && cd dependency_ws
fi

git clone --recursive https://github.com/stack-of-tasks/pinocchio &&
cd pinocchio && git checkout master &&
mkdir build && cd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 &&
make -j4 &&
sudo make install

echo " Configure environment variables @ ~/.bashrc"
echo " =========== Example =========== "
echo " export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH =/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH "
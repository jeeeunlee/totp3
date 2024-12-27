#! /bin/bash
PATH_PACKAGE="$(pwd)"

if [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt install build-essential cmake pkg-config git -y &&
    sudo apt install g++ python3-dev autotools-dev libicu-dev libbz2-dev libboost-all-dev -y &&
    sudo apt install libyaml-dev -y &&
    sudo apt install libyaml-cpp-dev -y &&
    sudo apt install libeigen3-dev -y &&
    sudo apt install liburdfdom-headers-dev liburdfdom-dev -y &&    
    sudo apt install libopenscenegraph-dev -y &&
    sudo apt install coinor-libclp-dev -y &&
    sudo apt install coinor-clp -y 
else
    echo "[error] OS not detected"
fi


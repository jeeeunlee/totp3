#! /bin/bash

echo "conda base"
eval "$(~/anaconda3/bin/conda shell.bash hook)"

echo "=============================="
echo "conda create environment 'DHC'"
echo "=============================="
ENVS=$(conda env list | awk '{print }' )
if [ conda env list = *"DHC"* ]; then
    conda env remove -n DHC
    conda create --name DHC python=3.8 -y
    conda activate DHC
else 
    conda create --name DHC python=3.8 -y
    conda activate DHC
    exit
fi;


echo "=============================="
echo "conda install eigenpy"
echo "=============================="
conda install eigenpy -c conda-forge -y

echo "=============================="
echo "conda install pybind11"
echo "=============================="
conda install -c conda-forge pybind11 -y

echo "=============================="
echo "conda install scipy"
echo "=============================="
conda install -c anaconda scipy -y

echo "=============================="
echo "conda install tqdm"
echo "=============================="
conda install -c conda-forge tqdm -y

echo "=============================="
echo "conda install pinocchio"
echo "=============================="
conda install pinocchio -c conda-forge -y


# echo "conda install gepetto-viewer"
# conda install gepetto-viewer gepetto-viewer-corba -c conda-forge

echo "=============================="
echo "conda install meshcat viewer"
echo "=============================="
conda install -c conda-forge meshcat-python -y

echo "=============================="
echo "conda install pybullet"
echo "=============================="
conda install -c conda-forge pybullet -y

echo "=============================="
echo "conda install opencv"
echo "=============================="
conda install -c conda-forge opencv -y

echo "=============================="
echo "conda install imageio"
echo "=============================="
conda install -c conda-forge imageio -y

echo "=============================="
echo "conda install keyboard"
echo "=============================="
conda install -c conda-forge keyboard -y

echo "=============================="
echo "conda install pyyaml"
echo "=============================="
conda install -c conda-forge pyyaml -y
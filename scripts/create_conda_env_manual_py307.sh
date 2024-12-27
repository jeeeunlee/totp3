#! /bin/bash

echo "conda base"
eval "$(~/anaconda3/bin/conda shell.bash hook)"

echo "=============================="
echo "conda create environment 'DHC37'"
echo "=============================="
ENVS=$(conda env list | awk '{print }' )
if [ conda env list = *"DHC37"* ]; then
    # exit
    # conda env remove -n DHC37
    # conda create --name DHC37 python=3.7 -y
    conda activate DHC37
else 
    conda create --name DHC37 python=3.7 -y
    conda activate DHC37
    
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

# echo "=============================="
# echo "conda install meshcat viewer"
# echo "=============================="
# conda install -c conda-forge meshcat-python -y

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

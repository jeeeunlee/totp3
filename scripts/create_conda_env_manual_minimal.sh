#! /bin/bash

echo "conda base"
eval "$(~/anaconda3/bin/conda shell.bash hook)"

echo "=============================="
echo "conda create environment 'DHC-minimal'"
echo "=============================="
ENVS=$(conda env list | awk '{print }' )
if [ conda env list = *"DHC-minimal"* ]; then
    exit
    conda env remove -n DHC-minimal
    conda create --name DHC-minimal python=3.7 -y
    conda activate DHC-minimal
else 
    conda create --name DHC-minimal python=3.7 -y
    conda activate DHC-minimal
    
fi;

echo "=============================="
echo "conda install pybind11"
echo "=============================="
conda install -c conda-forge pybind11 -y


echo "=============================="
echo "conda install numpy"
echo "=============================="
conda install numpy -c conda-forge -y
#! /bin/bash

echo "conda base"
eval "$(~/anaconda3/bin/conda shell.bash hook)"

echo "=============================="
echo "conda create environment 'DHC'"
echo "=============================="
conda create --name DHC -f env_dhc.yml
conda activate DHC
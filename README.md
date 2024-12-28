## Introduction

The code is developed based on
- [pnc](https://github.com/junhyeokahn/PnC)

We've tested this SW on Ubuntu 18.04, 20.04, 22.04 and the code has dependencies on
- [conda](https://www.anaconda.com/products/distribution#linux) ,
- [clp](https://github.com/coin-or/Clp) for LP solver ,
- [eigenpy](https://github.com/stack-of-tasks/eigenpy) for pinocchio ,
- [pinocchio](https://github.com/stack-of-tasks/pinocchio) ,
- [pybullet](https://pybullet.org/wordpress/) ,

which can be installed from the follwing instructions.

## Installation
### 0. Clone the git repository
```cd ~/path/to/install``` <br/>
```git clone --recursive https://github.com/jeeeunlee/DHC ```

### 1. Dependencies installation
- **Conda** installation<br/>
You need to install [conda](https://conda.io/projects/conda/en/latest/user-guide/install/linux.html)
- **Binary libraries** installation <br/>
```source scripts/install_dependencies.sh```
- **Eigenpy** installation <br/>
You are recommended to build from source [eigenpy](https://github.com/stack-of-tasks/eigenpy), but you can also install by running a script <br/>
```source scripts/install_eigenpy_from_src.sh```
- **Pinocchio** installation <br/>
You are recommended to build from source [pinocchio](https://github.com/stack-of-tasks/pinocchio). Looking at [pinocchio installation guide](https://stack-of-tasks.github.io/pinocchio/download.html) "Build from source" tab may be helpful. Or, you can also install by running a script <br/>
```source scripts/install_pinocchio_from_src.sh```

### 2. Virtual env setup via Conda
- for Ubuntu 22.04 & python == 3.10 <br/>
```source scripts/create_conda_env_manual_py310.sh```
- for Ubuntu 20.04 & python == 3.8 <br/>
```source scripts/create_conda_env_manual_py308.sh```
<!-- - env.yml installation <br/>
```source scripts/create_conda_env.sh``` -->

### 3. Compile and Run
- Initiate python env:<br/>
```conda activate DHC```
- Compile:<br/>
```mkdir build && cd build && cmake.. && make -j4 ```
- Run pybullet sim:<br/>
```cd .. && python src/simulator/main.py```

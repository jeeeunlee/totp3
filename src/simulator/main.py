import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/src/simulator/')

from config.pybullet_configuration import Config

if __name__ == "__main__":
    PYTHON_RUN_SCRIPT = cwd + '/src/simulator/'+ Config.PYTHON_RUN_SCRIPT
    exec(open(PYTHON_RUN_SCRIPT).read())

import numpy as np
import yaml


class Config(object):
    
    with open('config/manipulator/TEST_INTERFACE.yaml') as f:
        robCfg = yaml.load(f, Loader=yaml.FullLoader)     
        
    robot_type = 'rs020n'
    ROBOT_FILE_NAME = "/" + robCfg[robot_type]['robot_urdf']  
    PYTHON_RUN_SCRIPT = robCfg[robot_type]['robot_script']    
        
    if(robot_type == 'rs020n'):
        # RS020N7dof
        ROBOTDOF = 6
        INIT_JOINT_CONFIG = {'j1':2.22,
                            'j2':-0.63,
                            'j3':-2.4,
                            'j4': 0.,
                            'j5':-1.37,
                            'j6':0.91}    
            
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1
    CAMERA_DT = 0.05
    KP = 0.
    KD = 0.

    PRINT_TIME = False #True #False
    PRINT_ROBOT_INFO = False
    VIDEO_RECORD = False
    VIDEO_DIR = ''
    RECORD_FREQ = 50
    SIMULATE_CAMERA = False

    KP, KD = dict(), dict()


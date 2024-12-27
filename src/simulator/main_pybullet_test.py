import os
import sys


cwd = os.getcwd()
print(cwd)
sys.path.append(cwd)
sys.path.append(cwd + '/src/simulator/utils')
sys.path.append(cwd + '/src/simulator')
sys.path.append(cwd + '/build/lib')

key_mapping = {127: 'start'} #backspace

import time, math
from collections import OrderedDict
import copy
import signal
import shutil

import cv2
import pybullet as p
import numpy as np

np.set_printoptions(precision=2)

import utils.pybullet_util as pybullet_util
# import util

class JogController:
    def __init__(self, current_config, jog_scale):
        self.dq = jog_scale
        self.q = current_config
        self.jointnames = self.q.keys()
        
    def get_plus_jogged_position(self, jointname):
        self.q[jointname] += self.dq
        return self.q
    
    def get_minus_jogged_position(self, jointname):
        self.q[jointname] -= self.dq
        return self.q
    
    def test_all_generator(self, n_jog_max):
        for jointname in self.jointnames:            
            for i in range(n_jog_max):
                yield self.get_plus_jogged_position(jointname)                
            for i in range(250):
                yield self.q                
            for i in range(2*n_jog_max):
                yield self.get_minus_jogged_position(jointname)                
            for i in range(250):
                yield self.q                
            for i in range(n_jog_max):
                yield self.get_plus_jogged_position(jointname)                
            for i in range(250):
                yield self.q
            
    
    def test_motion_generator(self, jointname, n_jog_max):
        for i in range(n_jog_max):
            yield self.get_plus_jogged_position(jointname)            
        for i in range(250):
            yield self.q            
        for i in range(2*n_jog_max):
            yield self.get_minus_jogged_position(jointname)            
        for i in range(250):
            yield self.q            
        for i in range(n_jog_max):
            yield self.get_plus_jogged_position(jointname)            
        for i in range(250):
            yield self.q
            
        
class Config(object):
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1

    # RS020N
    ROBOT_FILE_NAME = "/robot_description/RS020N/rs020n.urdf"    
    INIT_JOINT_CONFIG = {'j1': 0.0,
                         'j2': 0.0,
                         'j3': 0.0,
                         'j4': 0.0,
                         'j5': 0.0,
                         'j6': 0.0}
    
    INITIAL_POS_WORLD_TO_BASEJOINT = [0, -0.0, 0.62]
    # INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0.7071, 0.7071]
    INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0.0, 1.0]
    
    TABLE_BASE_CONFIG = {'basePosition':[0.0,0.2,0] }
    
    # yellow
    CONVEYOR1_BASE_CONFIG = {'basePosition':[-0.15, -0.6, 0],
                             'baseOrientation':[0., 0., 0.0, 1.0]}
    # green
    CONVEYOR2_BASE_CONFIG = {'basePosition':[-1.1, 0.10, 0],
                             'baseOrientation': [0,0,0.7068252,0.7073883]}   


    PRINT_POSITION = False
    PRINT_TIME = False #True #False
    PRINT_ROBOT_INFO = True


if __name__ == "__main__":

    # Environment Setup
    p.connect(p.GUI,  options="--opengl2")
    p.resetDebugVisualizerCamera(cameraDistance=3.0,
                                 cameraYaw=-65,
                                 cameraPitch=-10,
                                 cameraTargetPosition=[0, 0.0, 1.0])
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                                numSubSteps=Config.N_SUBSTEP)


    # Create Robot, Ground, Environments
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(cwd + Config.ROBOT_FILE_NAME,
                       Config.INITIAL_POS_WORLD_TO_BASEJOINT,
                       Config.INITIAL_QUAT_WORLD_TO_BASEJOINT)

    p.loadURDF(cwd + "/robot_description/ground/plane.urdf", [0, 0, 0],
               useFixedBase=1)

    tableUid01 = p.loadURDF(cwd + "/robot_description/table/table.urdf",
                            basePosition=Config.TABLE_BASE_CONFIG['basePosition'])

    tableUid02 = p.loadURDF(cwd + "/robot_description/conveyor/conveyor.urdf",
                          basePosition=Config.CONVEYOR1_BASE_CONFIG['basePosition'])

    tableUid03 = p.loadURDF(cwd + "/robot_description/conveyor/conveyor.urdf",
                          basePosition=Config.CONVEYOR2_BASE_CONFIG['basePosition'],
                          baseOrientation=Config.CONVEYOR2_BASE_CONFIG['baseOrientation'])                      

    nq, nv, na, joint_id, link_id = pybullet_util.get_robot_config( robot, Config.PRINT_ROBOT_INFO)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    
    # Initial Config
    joint_positions = Config.INIT_JOINT_CONFIG
    pybullet_util.set_config(robot, joint_id, Config.INIT_JOINT_CONFIG)

    # Link Damping
    p.changeDynamics(robot, -1, linearDamping=0., angularDamping=0.)
    pybullet_util.set_link_damping(robot, link_id.values(), 0., 0.)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0.0)
    
    # draw link frames
    for k, v in link_id.items():
        pybullet_util.draw_link_frame(robot, v, linewidth=5.0, text=None)
    
    # jogger
    jog_period = 500
    jog_scale = (np.pi/2)/(jog_period)
    jc = JogController(joint_positions, jog_scale)
    joggen = jc.test_all_generator(0)
    
    # trajectory

    # Run Sim
    t = 0
    dt = Config.CONTROLLER_DT  
    count = 0 
    t_lastkeypressed = 0
    
    while (1):
        keys = pybullet_util.get_key_pressed()
        if(len(keys)>0 and t_lastkeypressed+1.0 < t):
            if(keys[0]==32):
                t_lastkeypressed = t
                print(t_lastkeypressed)
                joggen = jc.test_all_generator(jog_period)

        tstart = time.time()
        # # Get SensorData
        sensor_data_dict = pybullet_util.get_sensor_data(robot, joint_id)
        sensor_data_dict = pybullet_util.get_trq_data(robot, joint_id, sensor_data_dict)
        
        # Apply Trq
        # pybullet_util.set_motor_trq(robot, joint_id, command.joint_torques)
        joint_positions = next(joggen, joint_positions)
        pybullet_util.set_motor_pos(robot, joint_id, joint_positions)
        p.stepSimulation()

        # time.sleep(dt)
        tsleep = dt-(time.time()-tstart)
        if(tsleep>0):
            time.sleep(tsleep)
        if Config.PRINT_TIME:            
            print(time.time()-tstart)
        if Config.PRINT_POSITION:
            print(np.array(joint_positions.values()))
            print(np.array(sensor_data_dict['joint_pos']))
        t += dt
        count += 1


import numpy as np
import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/src/simulator/utils')
sys.path.append(cwd + '/src/simulator')
sys.path.append(cwd + '/build/lib')

import dhc.dhc_interface as dhc_interface

class MotionConfig():    
    def __init__(self, interface, TestMotions, dx=0, dy=0, dz=0):
        self.interface = interface
        
        # set cartesian way points        
        # 1. add motions to wpts
        self.motion_num = 0
        self.wpts_list = list()
        self.init_config_list = list()
        
        qwpts = dhc_interface.WPT_DATA() 
        for i, motion in enumerate(TestMotions.motion_sequence):
            self.motion_num += 1
            self.adjust_waypoints(dx, dy, dz, motion)
            
            wpt = dhc_interface.WPT_DATA()
            wpt.data = motion[1:]
            self.wpts_list.append(wpt)
            
            init_wpt = [motion[0]]         
            
            q0 = TestMotions.initial_configs[i]
            self.interface.solveIK(q0, init_wpt, qwpts)
            q0 = qwpts.data[0].tolist()
            q0 = self.adjust_lastjoint(q0)
            self.init_config_list.append(q0)            
                    
        # print(q0)
  

    def get_motion_num(self):
        return self.motion_num
    
    def get_init_config(self, i=0):
        # print(self.init_config)
        return self.init_config_list[i]
        
    def get_waypoints(self, i=0):
        # print("----get_waypoints----")
        # for wpt in self.wpts_list[i].data:
        #     print(wpt)
        # print("----get_waypoints----")
        return self.wpts_list[i]
    
    def adjust_waypoints(self, dx, dy, dz, wptsdata):
        for pt in wptsdata:
            pt[0] = pt[0] + dx 
            pt[1] = pt[1] + dy 
            pt[2] = pt[2] + dz 
        # return wptsdata
        
    def adjust_lastjoint(self, q):
        idx = len(q)
        if ( q[idx-1] > np.pi ):
                q[idx-1] = q[idx-1] - 2*np.pi
        elif( q[idx-1] < - np.pi ):
                q[idx-1] = q[idx-1] + 2*np.pi
        return q
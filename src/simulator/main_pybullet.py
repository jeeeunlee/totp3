from config.pybullet_configuration import Config
from config.pybullet_motion_random import TestMotions
from config.pybullet_motion import MotionConfig
from utils.pybullet_simulator import Simulator

import utils.pybullet_util as pybullet_util

motion_idx = 0

def main():    
    sim = Simulator(Config)       
    
    sim.gripdata.setObject(0.0, 0.4, 0.5) #m,l,l+h
    sim.interface.updateGripData(sim.gripdata)
    sim.interface.updateAccLimit([5.]*len(sim.joint_id))
    sim.interface.updateJerkLimit([1000.]*len(sim.joint_id))
    # sim.gripdata.setGripper(0.1, 0.1, 0.03, 0.7, 78) # r,h,rpad,mu,fs
        
    ## SET waypoints
    motioncfg = MotionConfig(sim.interface, 
                             TestMotions,
                             dx=0, dy=-0.6, dz=0)
    sim.set_init_config( motioncfg.get_init_config(motion_idx) )

    # Run Sim
    count_planning = 0 
    b_planning = True
    
    while (1):
        # planning 
        keys = pybullet_util.get_key_pressed()
        if(len(keys)>0):
            if(keys[0]==32 and b_planning):
        # if(count==1000):              
                print("key pressed")
                sim.interface.doPlanning( motioncfg.get_waypoints(motion_idx) )
                b_planning=False
                count_planning = sim.count                

        # if(b_planning == False and
        #     sim.count - count_planning == 500):
        #         sim.gripdata.setObject(0.0, 0.4, 0.5) #m,l,l+h
        #         sim.interface.updateGripData(sim.gripdata)
        #         sim.interface.rePlanning()

        sim.update_in_loop()
        
        
if __name__ == "__main__":
    main()
    

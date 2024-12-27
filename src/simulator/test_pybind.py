import os
import sys

import dhc.dhc_interface as dhc_interface

RS020N=0

if __name__ == "__main__":
        
    # Construct Interface
    interface = dhc_interface.TestInterface(RS020N)
    sensor_data = dhc_interface.SensorData(6)
    command = dhc_interface.RobotCommand(6)
    gripdata = dhc_interface.DexGripperData()
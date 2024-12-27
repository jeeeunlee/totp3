import os
import sys


cwd = os.getcwd()
print(cwd)
sys.path.append(cwd)
sys.path.append(cwd + '/build/lib')

import dhc.dhc_interface as dhc_interface

## robot type
RS020N=0

datafolder = "/home/jelee/my_ws/dex_log_analysis/data/August04/exp1/"
def read_traj(filename, trajdata):
    # trajdata : dhc_interface.TRAJ_DATA
    # tdata, qdata, dqdata, xdata, dxdata, period, singularityinpath
    
    with open(filename) as f:
        tdata = []
        qdata = []
        dqdata = []
        for l in f.readlines():
            l = [float(v) for v in l.split(',')]
            ndof = int((len(l)-1)/2)
            tdata.append(l[0])
            qdata.append(l[1:(ndof+1)])
            dqdata.append(l[(ndof+1):])
        
    trajdata.tdata = tdata
    trajdata.qdata = qdata
    trajdata.dqdata = dqdata


if __name__ == "__main__":
        
    # Construct Interface
    interface = dhc_interface.TestInterface(RS020N)
    trajdata = dhc_interface.TRAJ_DATA()
    gripdata = dhc_interface.DexGripperV4()

    filename = os.path.join(datafolder,"front_pick/5565_4_t_traj.txt")
    m = 1
    rcom = 0.243/2
    pcom = 0.243/2
    gripdata.setObject(m, rcom, pcom)
    read_traj(filename, trajdata)
    interface.getMaxLoadOnTrajectory(gripdata, trajdata)
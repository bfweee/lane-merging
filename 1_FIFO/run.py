import os
import sys
import random
from fifo import *
import numpy as np
import time
seednum=18

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    print('SUMO_HOME is In Environment!')
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumo_gui = 'sumo-gui'#
sumo_config = 'ramp.sumocfg'


def release_speed_limit():
    cavs = get_position_vehiclelist("E3_0", -655, -650)
    for v in cavs:
        if traci.vehicle.getSpeedFactor(v)<0.50:
            traci.vehicle.setSpeedFactor(v, factor=0.515)

def generate_veh_m(step,internal,c):#c=0不换道#1换道
    departlane = np.random.randint(0, 2)
    if step%internal==0.0:
        if c==1 and departlane==0:
            carid = "mainline" + "_" + str(departlane) + "_" + str(step)
            traci.vehicle.add(carid, "me", typeID='vtype', departLane=departlane,
                              departPos=str(np.random.uniform(0, 1) * 200),
                              departSpeed=16.5)  # 13str(random.uniform(0,1)*100))-trunc_gauss(4,2,2,4)
            traci.vehicle.setSpeedFactor(carid, factor=np.random.uniform(0.50+0.01,0.51+0.02))
            # traci.vehicle.setLaneChangeMode(carid, 0b011001010101)
            # traci.vehicle.setLaneChangeMode(carid, 0b011001000000)
            traci.vehicle.setLaneChangeMode(carid, 1621)

        elif c==1 and departlane==1:
            carid = "mainline" + "_" + str(departlane) + "_" + str(step)
            traci.vehicle.add(carid, "me", typeID='vtype', departLane=departlane,
                              departPos=str(np.random.uniform(0, 1) * 200),
                              departSpeed=16.5)  # 13str(random.uniform(0,1)*100))-trunc_gauss(0.1,0.1,0.50,0.53)
            traci.vehicle.setSpeedFactor(carid, factor=np.random.uniform(0.50+0.01,0.51+0.02))
            traci.vehicle.setLaneChangeMode(carid, 0b011000000100)
            # traci.vehicle.setLaneChangeMode(carid, 1621)
            # traci.vehicle.setLaneChangeMode(carid, 1621)
        else:


            carid = "mainline" + "_" + str(departlane) + "_" + str(step)
            traci.vehicle.add(carid, "me", typeID='vtype', departLane=departlane,
                              departPos=str(np.random.uniform(0, 1) * 200),
                              departSpeed=16.5)  # 13str(random.uniform(0,1)*100))-trunc_gauss(4,2,2,4)
            traci.vehicle.setSpeedFactor(carid, factor=np.random.uniform(0.50+0.01,0.51+0.02))
            traci.vehicle.setLaneChangeMode(carid, 0b000000000000)
            # if departlane<3:
            #     carid = "mainline" + "_" + str(0) + "_" + str(step)
            #     traci.vehicle.add(carid, "me", typeID='vtype', departLane=0,
            #                       departPos=str(np.random.uniform(0, 1) * 200),
            #                       departSpeed=16.5)  # 13str(random.uniform(0,1)*100))-trunc_gauss(4,2,2,4)
            #     traci.vehicle.setSpeedFactor(carid, factor=np.random.uniform(0.50+0.01,0.51+0.02))
            #     traci.vehicle.setLaneChangeMode(carid, 0b000000000000)
            # else:
            #     carid = "mainline" + "_" + str(1) + "_" + str(step)
            #     traci.vehicle.add(carid, "me", typeID='vtype', departLane=1,
            #                       departPos=str(np.random.uniform(0, 1) * 200),
            #                       departSpeed=16.5)  # 13str(random.uniform(0,1)*100))-trunc_gauss(4,2,2,4)
            #     traci.vehicle.setSpeedFactor(carid, factor=np.random.uniform(0.50 + 0.01, 0.51 + 0.02))
            #     traci.vehicle.setLaneChangeMode(carid, 0b000000000000)
            #


def generate_veh_r(step, internal, c):  # c=0不换道#1换道
    if step%internal==0.0:
        carid = "rampline"+"_"+str(step)
        traci.vehicle.add(carid, "re", typeID='vtype',departLane=0,departPos=str(200+random.uniform(0,1)*200),departSpeed=10)#str(random.uniform(0,1)*100)13-trunc_gauss(4,2,2,4)
        # traci.vehicle.setSpeedMode(carid, 37)
        # traci.vehicle.setSpeedFactor(carid,factor=random.uniform(0.50,0.51))
        traci.vehicle.setSpeedFactor(carid, factor=random.uniform(0.30+0.01, 0.31+0.01))
        if c == 0:
            traci.vehicle.setLaneChangeMode(carid, 0b000000000000)
        else:
            traci.vehicle.setLaneChangeMode(carid, 1621)





def run_fifo(mianlie_flow=3000, ramp_line=800, simulation_time=6000):

    traci.start([
        sumo_gui,
        "-c", sumo_config,
        "--no-warnings", "t",
        "--seed", "2",
        # "--lateral-resolution", "0.1",
        # "--lanechange.duration","0.3",
    ])
    rampc = Ramp_control()
    interal_m = 36000 // mianlie_flow
    interal_r = 36000 // ramp_line
    t = 0
    np.random.seed(seednum+3)
    random.seed(seednum + 5)
    generate_veh_r(t, interal_r, 0)
    generate_veh_m(t, interal_m, 0)
    traci.simulationStep()
    t = 1
    while traci.simulation.getMinExpectedNumber() > 0:
        if t <= simulation_time:
            generate_veh_r(t, interal_r, 0)
            generate_veh_m(t, interal_m,0)
        release_speed_limit()
        rampc.run_fifo(-500,"only")
        traci.simulationStep()
        t = t + 1
    traci.close()

if __name__ == '__main__':
    run_fifo(mianlie_flow=3600, ramp_line=1000, simulation_time=9000)
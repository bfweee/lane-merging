import math
import traci
import matplotlib.pyplot as plt
import numpy as np

def has_duplicate(lst1, lst2):
    set1 = set(lst1)
    set2 = set(lst2)
    return bool(set1.intersection(set2))

#获取车辆当前位置通过合流点最短的位置。
def getmin_travel_time(v0,p0,vmax,ev,ep,amax=3,amin=4):
    return (vmax**2-v0**2)/(amax*(v0+vmax))+\
           (vmax**2-ev**2)/(amin*(ev+vmax))\
           +(ep-p0-(vmax**2-v0**2)/(2*amax)-(vmax**2-ev**2)/(2*amin))/vmax

'''
功能：获取指定位置的车辆列表（车辆在前的在列表前面）
输入：车道名称，终止位置
输出：指定位置的车辆列表
'''
def get_position_vehiclelist(lane_name,pos,poe):
    vehiclelist = []
    lane_vlist = [ele for ele in reversed(traci.lane.getLastStepVehicleIDs(lane_name))]
    if len(lane_vlist)>0:
        for i in lane_vlist:
            if traci.vehicle.getPosition(i)[0]<=poe and traci.vehicle.getPosition(i)[0]>=pos:
                vehiclelist.append(i)
    return vehiclelist



#快速排序法
def re_quick_sort(arr):
    def quick_sort(arr):  # print(quick_sort([3, 6, 8, 10, 1, 2, 1]))  # 输出: [1, 1, 2, 3, 6, 8, 10]
        if len(arr) <= 1:
            return arr
        pivot = traci.vehicle.getPosition(arr[len(arr) // 2])[0]
        left = [x for x in arr if traci.vehicle.getPosition(x)[0] < pivot]
        middle = [x for x in arr if traci.vehicle.getPosition(x)[0] == pivot]
        right = [x for x in arr if traci.vehicle.getPosition(x)[0] > pivot]
        return quick_sort(left) + middle + quick_sort(right)
    result = quick_sort(arr)
    return [ele for ele in reversed(result)]


class Ramp_control():
    def __init__(self ,steps=200,rollout_times=20):
        self.ep = 100
        self.w1=1#加速度大小a2
        self.w2=1#加速度变化量u2
        self.ev = 17.5
        self.ea = 0
        self.W = math.sqrt(self.w2/self.w1)
        self.info_v = {}#车辆ID：［ct,sp,sv,sa,et,[加速度]，[速度],[位置]，车道id］
        self.dv = 18
        self.changetime =1.5#1.5
        self.followtime =0.9 #1.2
        self.dt = 0.1
        self.a=[]
        self.ti=[]
        self.lastcontrol=[]
        # self.strat_c=-250#单车道实验
        self.strat_c = -150

    def get_v(self):
        return self.info_v.keys()
        # 监测是否在控制时间内

    def getindex(self, x, l):
        if x in l:
            index = l.index(x)
            return index
        else:
            return 1000.1

    def hamilton(self, sp, sv, sa, st, et):

        R = np.zeros((6, 6))
        R[0, :] = [np.exp(self.W * st), np.exp(-self.W * st), (1 / self.w2) * st, -1 / self.w2, 0, 0]
        R[1, :] = [self.W ** (-1) * np.exp(self.W * st), -self.W ** (-1) * np.exp(-self.W * st),
                   (1 / 2) * (1 / self.w2) * st ** (2), -1 / self.w2 * st, 1, 0]
        R[2, :] = [self.W ** (-2) * np.exp(self.W * st), self.W ** (-2) * np.exp(-self.W * st),
                   (1 / 6) * (1 / self.w2) * st ** (3), -(1 / 2) * 1 / self.w2 * st ** (2), st, 1]
        R[3, :] = [np.exp(self.W * et), np.exp(-self.W * et), (1 / self.w2) * et, -1 / self.w2, 0, 0]
        R[4, :] = [self.W ** (-1) * np.exp(self.W * et), -self.W ** (-1) * np.exp(-self.W * et),
                   (1 / 2) * (1 / self.w2) * et ** (2), -1 / self.w2 * et, 1, 0]
        R[5, :] = [self.W ** (-2) * np.exp(self.W * et), self.W ** (-2) * np.exp(-self.W * et),
                   (1 / 6) * (1 / self.w2) * et ** (3), -(1 / 2) * 1 / self.w2 * et ** (2), et, 1]

        P = np.array([[sa], [sv], [sp], [self.ea], [self.ev], [self.ep]])

        try:
            C = np.linalg.inv(R) @ P
        except np.linalg.LinAlgError:
            C = np.linalg.pinv(R) @ P
        return C

    def get_max_et(self):
        last_et = -1
        last_v = "0"
        for i in self.info_v.keys():
            if last_et <= self.info_v[i][4]:
                last_et = self.info_v[i][4]
                last_v = i
        return last_v, last_et

    def run_fifo(self, s, model="only"):

        leave_set = get_position_vehiclelist("E3_0", self.strat_c-10,self.strat_c) + get_position_vehiclelist("E0_0", self.strat_c-10, self.strat_c)
        thro_sit = []
        if len(leave_set) != 0 and model == "only":
            for i in leave_set:
                if i not in self.info_v.keys():  # 当有 没有规划的车即将走出规划区域的时候，进行控制：
                    last_v,last_et=self.get_max_et()
                    m_sit = re_quick_sort(get_position_vehiclelist("E0_0", s, self.strat_c))
                    r_sit = re_quick_sort(get_position_vehiclelist("E3_0", s, self.strat_c))
                    v_in_info_m = []
                    for v in m_sit:
                        if v in self.info_v.keys():
                            v_in_info_m.append(v)
                    for v in v_in_info_m:
                        m_sit.remove(v)

                    v_in_info_r = []
                    for v in r_sit:
                        if v in self.info_v.keys():
                            v_in_info_r.append(v)
                    for v in v_in_info_r:
                        r_sit.remove(v)
                    thro_sit=m_sit+r_sit
                    thro_sit=re_quick_sort(thro_sit[:])
                    print(thro_sit)
                    break
        elif model == "two":
            last_v, last_et = self.get_max_et()
            m_sit = re_quick_sort(get_position_vehiclelist("E0_0", s, self.strat_c))
            r_sit = re_quick_sort(get_position_vehiclelist("E3_0", s, self.strat_c))
            v_in_info_m = []
            for v in m_sit:
                if v in self.info_v.keys():
                    v_in_info_m.append(v)
            for v in v_in_info_m:
                m_sit.remove(v)
            v_in_info_r = []
            for v in r_sit:
                if v in self.info_v.keys():
                    v_in_info_r.append(v)
            for v in v_in_info_r:
                r_sit.remove(v)
            thro_sit = m_sit + r_sit
            thro_sit = re_quick_sort(thro_sit[:])


        ct = traci.simulation.getTime()
        for veh_index in range(len(thro_sit)):
            # print(veh_index)
            if veh_index == 0:
                if last_et == -1:
                    et = getmin_travel_time(v0=traci.vehicle.getSpeed(thro_sit[veh_index]),p0=traci.vehicle.getPosition(thro_sit[veh_index])[0],vmax=self.dv,ev=self.ev,ep=self.ep) + ct
                    et = float(format(et, '.1f'))
                    # 车辆ID：［ct,sp,sv,sa,et,[加速度]，[速度],[位置]，[时间]车道id］
                    sp = traci.vehicle.getPosition(thro_sit[veh_index])[0]
                    sv = traci.vehicle.getSpeed(thro_sit[veh_index])
                    sa = traci.vehicle.getAcceleration(thro_sit[veh_index])
                    self.info_v[thro_sit[veh_index]] = [ct, sp, sv, sa, et, [sa], [sv], [sp], [ct],
                                                        traci.vehicle.getLaneIndex(thro_sit[veh_index])]
                else:
                    if traci.vehicle.getLaneID(thro_sit[veh_index]) == traci.vehicle.getLaneID(last_v):
                        et = max(last_et + self.followtime,
                                 ct + getmin_travel_time(v0=traci.vehicle.getSpeed(thro_sit[veh_index]),p0=traci.vehicle.getPosition(thro_sit[veh_index])[0],vmax=self.dv,ev=self.ev,ep=self.ep))
                        et = float(format(et, '.1f'))
                        # 车辆ID：［ct,sp,sv,sa,et,[加速度]，[速度],[位置]，[时间]车道id］
                        sp = traci.vehicle.getPosition(thro_sit[veh_index])[0]
                        sv = traci.vehicle.getSpeed(thro_sit[veh_index])
                        sa = traci.vehicle.getAcceleration(thro_sit[veh_index])
                        self.info_v[thro_sit[veh_index]] = [ct, sp, sv, sa, et, [sa], [sv], [sp], [ct],
                                                            traci.vehicle.getLaneIndex(thro_sit[veh_index])]
                    else:
                        et = max(last_et + self.changetime,
                                 ct + getmin_travel_time(v0=traci.vehicle.getSpeed(thro_sit[veh_index]),p0=traci.vehicle.getPosition(thro_sit[veh_index])[0],vmax=self.dv,ev=self.ev,ep=self.ep))
                        et = float(format(et, '.1f'))
                        # 车辆ID：［ct,sp,sv,sa,et,[加速度]，[速度],[位置]，[时间]车道id］
                        sp = traci.vehicle.getPosition(thro_sit[veh_index])[0]
                        sv = traci.vehicle.getSpeed(thro_sit[veh_index])
                        sa = traci.vehicle.getAcceleration(thro_sit[veh_index])
                        self.info_v[thro_sit[veh_index]] = [ct, sp, sv, sa, et, [sa], [sv], [sp], [ct],
                                                            traci.vehicle.getLaneIndex(thro_sit[veh_index])]
            else:
                if thro_sit[veh_index] in m_sit:
                    Lset = m_sit
                    Cset = r_sit
                elif thro_sit[veh_index] in r_sit:
                    Lset = r_sit
                    Cset = m_sit

                if thro_sit[veh_index - 1] in Lset:
                    et = max(self.info_v[thro_sit[veh_index - 1]][4] + self.followtime,
                             ct + getmin_travel_time(v0=traci.vehicle.getSpeed(thro_sit[veh_index]),p0=traci.vehicle.getPosition(thro_sit[veh_index])[0],vmax=self.dv,ev=self.ev,ep=self.ep))  # +(traci.vehicle.getPosition(thro_sit[veh_index-1])[0]-traci.vehicle.getPosition(thro_sit[veh_index])[0])/(self.dv+6)
                    et = float(format(et, '.1f'))
                    # 车辆ID：［ct,sp,sv,sa,et,[加速度]，[速度],[位置]，[时间]车道id］
                    sp = traci.vehicle.getPosition(thro_sit[veh_index])[0]
                    sv = traci.vehicle.getSpeed(thro_sit[veh_index])
                    sa = traci.vehicle.getAcceleration(thro_sit[veh_index])
                    self.info_v[thro_sit[veh_index]] = [ct, sp, sv, sa, et, [sa], [sv], [sp], [ct],
                                                        traci.vehicle.getLaneIndex(thro_sit[veh_index])]
                elif thro_sit[veh_index - 1] in Cset:
                    et = max(self.info_v[thro_sit[veh_index - 1]][4] + self.changetime,
                             ct + getmin_travel_time(v0=traci.vehicle.getSpeed(thro_sit[veh_index]),p0=traci.vehicle.getPosition(thro_sit[veh_index])[0],vmax=self.dv,ev=self.ev,ep=self.ep))  # +(traci.vehicle.getPosition(thro_sit[veh_index-1])[0]-traci.vehicle.getPosition(thro_sit[veh_index])[0])/(self.dv+6)
                    # 车辆ID：［ct,sp,sv,sa,et,[加速度]，[速度],[位置]，[时间]车道id］
                    et = float(format(et, '.1f'))
                    sp = traci.vehicle.getPosition(thro_sit[veh_index])[0]
                    sv = traci.vehicle.getSpeed(thro_sit[veh_index])
                    sa = traci.vehicle.getAcceleration(thro_sit[veh_index])
                    self.info_v[thro_sit[veh_index]] = [ct, sp, sv, sa, et, [sa], [sv], [sp], [ct],
                                                        traci.vehicle.getLaneIndex(thro_sit[veh_index])]

        if len(self.info_v.keys()) != 0:
            for i in self.info_v.keys():
                if traci.vehicle.getSpeedFactor(i) < 0.7:
                    traci.vehicle.setSpeedFactor(i, traci.vehicle.getSpeedFactor(i) + 0.4+0.2)
                if traci.vehicle.getTau(i) > 0.55:
                    traci.vehicle.setTau(i, 0.2)

        for veh in self.info_v.keys():
            if len(self.info_v[veh][5]) == 1:
                timeRange = np.arange(self.info_v[veh][0] - self.info_v[veh][0],
                                      self.info_v[veh][4] + self.dt - self.info_v[veh][0], self.dt)

                self.info_v[veh][8] = ["{:.3f}".format(num) for num in
                                       np.arange(self.info_v[veh][0], self.info_v[veh][4] + self.dt, self.dt).tolist()]
                self.info_v[veh][7] = [0 for i in timeRange.tolist()]
                # print(veh,self.info_v[veh][7],self.info_v[veh])
                self.info_v[veh][7][0] = self.info_v[veh][1]
                self.info_v[veh][6] = [0 for i in timeRange.tolist()]
                self.info_v[veh][6][0] = self.info_v[veh][2]
                self.info_v[veh][5] = [0 for i in timeRange.tolist()]
                self.info_v[veh][5][0] = self.info_v[veh][3]
                for j in range(len(timeRange) - 1):
                    c1, c2, c3, c4, c5, c6 = self.hamilton(self.info_v[veh][7][j], self.info_v[veh][6][j],
                                                           self.info_v[veh][5][j], timeRange[j],
                                                           self.info_v[veh][4] - self.info_v[veh][0])
                    self.info_v[veh][5][j + 1] = \
                        (c1 * np.exp(self.W * (timeRange[j + 1])) + c2 * np.exp(-self.W * (timeRange[j + 1])) + c3 * (
                                1 / self.w2) * (timeRange[j + 1]) - c4 * 1 / self.w2)[0].tolist()
                    if abs(self.info_v[veh][5][j + 1]) > 10:
                        self.info_v[veh][5][j + 1] = 0
                    self.info_v[veh][6][j + 1] = self.info_v[veh][6][j] + self.info_v[veh][5][j] * self.dt
                    self.info_v[veh][7][j + 1] = self.info_v[veh][7][j] + self.info_v[veh][6][j] * self.dt + \
                                                 self.info_v[veh][5][j] * self.dt * self.dt * 0.5

        vl0 = get_position_vehiclelist("E1_0", self.ep, self.ep + 5)
        if len(vl0) != 0:
            for cc in vl0:
                # traci.vehicle.changeSublane(cc, 3)
                traci.vehicle.changeLane(cc, 1, 1)

        ct = traci.simulation.getTime()

        for c in self.info_v.keys():
            index = self.getindex("{:.3f}".format(ct), self.info_v[c][8])
            if index != 1000.1:
                traci.vehicle.setAcceleration(c, self.info_v[c][5][index], self.dt)
                if self.info_v[c][5][index] > 0:
                    traci.vehicle.setColor(c, (255, 0, 0))
                elif self.info_v[c][5][index] < 0:
                    traci.vehicle.setColor(c, (0, 255, 0))

                traci.vehicle.setSpeed(c, self.info_v[c][6][index])
                # traci.vehicle.setAcceleration(c, self.info_v[c][5][index], 0.1)
                #
                # traci.vehicle.setAcceleration(c, self.info_v[c][5][index], self.dt)
                # traci.vehicle.setSpeed(c, self.info_v[c][6][index])

                # if c=='rampline_2760':
                #     print(ct,self.info_v[c][5][index],self.info_v[c][8][index],self.info_v[c][6][index])
                #     print(traci.vehicle.getAcceleration(c))


        # if 'mainline_0_540' in self.info_v.keys():
        #     print('mainline_0_540',self.info_v['mainline_0_540'][0:5])
        # if 'mainline_0_560' in self.info_v.keys():
        #     print('mainline_0_560',self.info_v['mainline_0_560'][0:5])
        # if 'mainline_0_2840' in self.info_v.keys():
        #     print('mainline_0_2840',self.info_v['mainline_0_2840'][0:5])
        # if 'mainline_0_2970' in self.info_v.keys():
        #     print('mainline_0_2970',self.info_v['mainline_0_2970'][0:5])
        # if 'rampline_2970' in self.info_v.keys():
        #     print('rampline_2970',self.info_v['rampline_2970'][0:5])
            # plt.plot(self.info_v['rampline_2970'][8],self.info_v['rampline_2970'][6])
            # plt.show()

        vl1 = get_position_vehiclelist("E1_1", self.ep + 10, self.ep + 15)
        vl0 = get_position_vehiclelist("E1_0", self.ep + 10, self.ep + 15)
        if len(vl1 + vl0) != 0:
            for i in vl1 + vl0:
                if i in self.info_v.keys():
                    del self.info_v[i]
        if len(vl1) != 0:
            for i in vl1:
                if traci.vehicle.getSpeedFactor(i) > 0.65:
                    traci.vehicle.setSpeedFactor(i, 0.51)

        if len(vl0) != 0:
            for i in vl0:
                if traci.vehicle.getSpeedFactor(i) > 0.65:
                    traci.vehicle.setSpeedFactor(i, 0.51)

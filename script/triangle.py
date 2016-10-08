# -*- coding:utf-8 -*-
# Create by steve in 16-10-7 at 下午4:25

import scipy as sp
import numpy as np

from scipy.optimize import minimize
from scipy.optimize import root

import matplotlib.pyplot as plt


class triangle:
    def __init__(self, beacon_info, beacon_set):
        self.beacon_pose = np.zeros_like(beacon_info[:, 0:2])
        self.beacon_pose = beacon_info[:, 0:2]

        self.beacon_range = np.zeros_like(beacon_info[:, 3:6])
        self.beacon_range = beacon_info[:, 3:6]

        self.beacon_set = beacon_set

        self.result = np.zeros_like(self.beacon_pose)

        self.the_range = np.zeros(3)

        self.ign = 10

    def setRealvar(self, ground_truth):
        self.gt = ground_truth

    def localization(self):
        for i in range(self.beacon_pose.shape[0]):
            self.the_range = self.beacon_range[i, :]
            if i < 1:
                self.result[i, :] = self.get_pose(self.gt[0, :])
            else:
                self.result[i, :] = self.get_pose(self.result[i - 1, :])
                # print(np.linalg.norm(self.result[i, :] - self.beacon_pose[i, :]))

        return self.result

    def get_pose(self, default_pose):
        '''
        compute pose based on the three distance
        :param dis_array:
        :return:
        '''

        re_pose = np.zeros(2)

        tmp_pose = minimize(self.cost_func,
                            # default_pose[0:2],
                            default_pose[0:2],
                            # method='Newton-CG',
                            jac=False)
        if tmp_pose.fun < 0.1:
            re_pose = tmp_pose.x[0:2]
            print(tmp_pose.fun)
        else:
            mul_re = np.zeros([3, 3])
            for i in range(3):
                self.ign = i
                tmp_pose = minimize(self.cost_func,
                                    # default_pose[0:2],
                                    default_pose[0:2],
                                    method='L-BFGS-B',
                                    bounds=((default_pose[0] - 1.0, default_pose[0] + 1.0),
                                            (default_pose[1] - 1.0, default_pose[1] + 1.0)),
                                    jac=False)
                mul_re[i, 0:2] = tmp_pose.x[0:2]
                mul_re[i, 2] = tmp_pose.fun
            self.ign = 4
            min_index = np.argmin(mul_re[:, 2])

            re_pose = mul_re[min_index, 0:2]
            print(mul_re[min_index,2])


        return re_pose

    def cost_func(self, pose):
        dis = np.zeros(3)


        t_pose = np.zeros(3)
        t_pose[0:2] = pose
        t_pose[2] = 1.12

        for i in range(3):

            dis[i] = np.linalg.norm(t_pose - self.beacon_set[i, :])
            if i == self.ign:
                dis[i] = 0.0
        return np.linalg.norm(dis - self.the_range)

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

    def setRealvar(self, ground_truth):
        self.gt = ground_truth

    def localization(self):
        for i in range(self.beacon_pose.shape[0]):
            self.the_range = self.beacon_range[i, :]
            self.result[i, :] = self.get_pose(self.gt[i, :])
            print(np.linalg.norm(self.result[i, :] - self.beacon_pose[i, :]))

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
                            [4.0, 4.0],
                            # method='Newton-CG',
                            jac=False)

        re_pose = tmp_pose.x[0:2]

        return re_pose

    def cost_func(self, pose):
        dis = np.zeros(3)

        t_pose = np.zeros(3)
        t_pose[0:2] = pose
        t_pose[2] = 1.12

        for i in range(3):
            dis[i] = np.linalg.norm(t_pose - self.beacon_set[i, :])

        return np.linalg.norm(dis - self.the_range)

# -*- coding:utf-8 -*-
# Create by steve in 16-10-7 at 下午4:25

import scipy as sp
import numpy as np

from scipy.optimize import minimize

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

        self.ign = 10000

        self.z_offset = 1.12

    def setz_Offset(self,zoff):
        self.z_offset = zoff


    def setRealvar(self, ground_truth):
        self.gt = ground_truth

    def localization(self):
        for i in range(self.beacon_pose.shape[0]):
            self.the_range = self.beacon_range[i, :]
            if i < 1:
                self.result[i, :] = self.get_pose((1.98119, 4.07225))
            else:
                self.result[i, :] = self.get_pose(self.result[i - 1, :])
                # print(np.linalg.norm(self.result[i, :] - self.beacon_pose[i, :]))

        return self.result

    def get_pose(self, default_pose):
        '''
        compute pose based on the three distance
        :param default_pose:
        :return:
        '''

        re_pose = np.zeros(2)

        tmp_pose = minimize(self.cost_func,
                            # default_pose[0:2],
                            default_pose[0:2],
                            # method='Newton-CG',
                            jac=False)
        if tmp_pose.fun < 0.35:
            re_pose = tmp_pose.x[0:2]
            # print(tmp_pose.fun)
        else:
            mul_re = np.zeros([3, 3])
            for i in range(3):
                self.ign = i

                dis_range = 0.5
                tmp_pose = minimize(self.cost_func,
                                    # default_pose[0:2],
                                    default_pose[0:2],
                                    method='L-BFGS-B',
                                    bounds=((default_pose[0] - dis_range, default_pose[0] + dis_range),
                                            (default_pose[1] - dis_range, default_pose[1] + dis_range)),
                                    jac=False)
                mul_re[i, 0:2] = tmp_pose.x[0:2]
                mul_re[i, 2] = tmp_pose.fun
            self.ign = 1000
            min_index = np.argmin(mul_re[:, 2])
            if mul_re[min_index, 2] > 0.5 and np.linalg.norm(mul_re[min_index, 0:2] - default_pose[0:2]) > 3.0:

                for i in range(3):
                    self.ign = i

                    dis_range = 15.0
                    tmp_pose = minimize(self.simple_cost_func,
                                        # default_pose[0:2],
                                        default_pose[0:2],
                                        method='L-BFGS-B',
                                        bounds=((default_pose[0] - dis_range, default_pose[0] + dis_range),
                                                (default_pose[1] - dis_range, default_pose[1] + dis_range)),
                                        jac=False)
                    mul_re[i, 0:2] = tmp_pose.x[0:2]
                    mul_re[i, 2] = tmp_pose.fun
                self.ign = 1000

            re_pose = mul_re[min_index, 0:2]
            # print(mul_re[min_index,2])

        return re_pose

    def cost_func(self, pose):
        dis_err = np.zeros(3)

        t_pose = np.zeros(3)
        t_pose[0:2] = pose
        t_pose[2] = self.z_offset
        tmp_sum = np.sum(self.the_range)

        for i in range(self.beacon_set.shape[0]):

            dis_err[i] = (np.linalg.norm(t_pose - self.beacon_set[i, :]) - self.the_range[i]) / np.sqrt(
                self.the_range[i] + 0.000001)

            if self.ign > self.beacon_set.shape[0]:
                if dis_err[i] < 0.0:
                    dis_err[i] *= 0.8
                    tmp_sum -= self.the_range[i] * 0.8
                else:
                    dis_err[i] *= 1.0


            if i == self.ign:
                dis_err[i] = 0.0
                tmp_sum -= self.the_range[i]
                # TODO!!!! check normalize parameter in this equation.
        return np.linalg.norm(dis_err) * tmp_sum

    def simple_cost_func(self, pose):
        dis = np.zeros(3)

        t_pose = np.zeros(3)
        t_pose[0:2] = pose
        t_pose[2] = self.z_offset

        for i in range(self.beacon_set.shape[0]):
            dis[i] = np.linalg.norm(t_pose - self.beacon_set[i, :]) - self.the_range[i]

        return np.linalg.norm(dis)

# -*- coding:utf-8 -*-
# Create by steve in 16-10-7 at 上午10:10

import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

from filter_fram import filter_frame

from PFONE import PFONE


class RangPf(filter_frame):
    def __init__(self):
        filter_frame.__init__(self)
        self.pfone = PFONE(2, 3, 100)

    def setInput(self, beacon_info, beacon_set):
        self.beacon_pose = beacon_info[:, 0:2]

        self.beacon_range = beacon_info[:, 3:6]
        self.beacon_set = beacon_set

        self.filter_result = np.zeros_like(self.beacon_pose)



    def initial_filter(self, N_praticles):
        '''
        Initial filter parameter and set some practicles.
        :param N_praticles:
        :return:
        '''

        self.pfone.reset(self.beacon_pose.shape[1], self.beacon_set.shape[0], N_praticles)
        self.pose_num = self.beacon_pose.shape[1]
        self.range_num = self.beacon_range.shape[1]
        self.all_result = np.zeros([self.beacon_pose.shape[0], self.pose_num + self.range_num])
        self.pfone.setBeaconPose(self.beacon_set)
        self.pfone.setPFParameter(0.3, 0.3, 1.12)
        return True

    def filter(self):
        '''
        main
        :return:
        '''
        # self.initial_filter(100)

        self.pfone.InitialValue(self.beacon_range[0, :])
        for i in range(self.beacon_range.shape[0]):
            if i % 30 == 0:
                print("finished", i * 1.0 / self.beacon_range.shape[0])

            # Sampling
            if i < 2:
                delta_vec = np.zeros(self.pose_num + self.range_num)
                # for j in range(delta_vec.shape[0]):
                #     delta_vec[j] = np.random.normal(0.0, self.pfone.state_var.mean() * 0.3)
                self.pfone.StateEqu(delta_vec, self.beacon_range[i, :])
            else:
                delta_vec = self.all_result[i - 1, :] - self.all_result[i - 2, :]
                # delta_vec[0:2] = delta_vec[0:2] / np.linalg.norm(delta_vec[0:2]) * 0.01
                # delta_vec[2:] = delta_vec[2:] / np.linalg.norm(delta_vec[2:])
                delta_vec = np.zeros_like(delta_vec)
                self.pfone.StateEqu(delta_vec, self.beacon_range[i, :])

            # Evaluated
            self.pfone.ObserveEva(self.beacon_range[i, :])

            # get Result
            self.all_result[i, :] = self.pfone.GetResult()

            # Resample
            self.pfone.ReSample()

        self.filter_result = self.all_result[:, 0:2]

        return self.filter_result

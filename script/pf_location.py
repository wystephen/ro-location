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
        # self.pfone = PFONE(2,3,100)

    # def setInput(self, beacon_info, beacon_set):
    #     self.beacon_pose = beacon_info[:,0:2]
    #
    #     self.beacon_range = beacon_info[:,3:6]
    #     self.beacon_set = beacon_set
    #
    #
    #     self.filter_result = np.zeros_like(self.beacon_pose)



    def initial_filter(self, N_praticles):
        '''
        Initial filter parameter and set some practicles.
        :param N_praticles:
        :return:
        '''

        self.pfone(self.beacon_pose.shape[1], self.beacon_set.shape[0], N_praticles)
        self.pfone.setBeaconPose(self.beacon_set)
        self.pfone.setPFParameter(0.5, 0.5, 1.12)










        return True

    def filter(self):
        '''
        main
        :return:
        '''
        self.initial_filter(100)

        self.pfone.InitialValue(self.beacon_range[0, :])



        return self.filter_result

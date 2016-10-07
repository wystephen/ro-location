# -*- coding:utf-8 -*-
# Create by steve in 16-10-7 at 下午2:59

import numpy as np
import scipy as sp


class filter:
    def __init__(self):
        self.pose = np.zeros(3)
        self.last_pose = np.zeros(3)

    def setInput(self,beacon_info,beacon_set):
        self.beacon_pose = beacon_info[:,0:2]
        self.beacon_range = beacon_info[:,3:6]
        self.beacon_set = beacon_set

        self.filter_result = np.zeros_like(self.beacon_pose)

    def filter(self):
        for i in range(self.beacon_pose.shape[0]):
            if i < 3:
                self.filter_result[i,:] = self.beacon_pose[i,:]
            else:
                if np.linalg.norm(self.filter_result[i-1,:]-self.beacon_pose[i,:]) > 15.0:
                    self.filter_result[i,:] = 2 * self.filter_result[i-1,:] - self.filter_result[i-2,:]
                else:
                    self.filter_result[i,:]  = self.beacon_pose[i,:]

        return self.filter_result



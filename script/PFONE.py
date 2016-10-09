# -*- coding:utf-8 -*-
# Create by steve in 16-10-9 at 上午9:38

import numpy as np
import scipy as sp

from scipy.optimize import minimize


class PFONE:
    def __init__(self, position_num,
                 beacon_num,
                 particle_num
                 ):
        '''
        This function is only use to define some value for filter.
        :position_num
        :beacon_num
        :particle_num
        :return:
        '''
        self.sample_vector = np.zeros([particle_num, position_num + beacon_num])
        self.weight_vector = np.ones([particle_num, 1])

        self.history_pose = np.zeros([2, position_num])
        self.history_range = np.zeros([2, beacon_num])

        for i in range(position_num):
            self.history_pose[0, i] = np.random.normal(0.0, 1.0)
        for i in range(beacon_num):
            self.history_range[0, i] = np.random.normal(0.0, 1.0)

    def setPFParameter(self, pose_var=0.5, beacon_var=0.5, z_offset=1.12):
        self.pose_var = pose_var
        self.beacon_var = beacon_var
        self.z_offset = z_offset

    def setBeaconPose(self, beaconpose):
        '''
        Pose of beacons.

        :param beaconpose:
        :return:
        '''
        self.beaconPose = beaconpose
        self.currentRange = np.zeros(beaconpose.shape[0])

        return True

    # def InitialValue(self,beacon_range):
    #     '''
    #
    #     :param beacon_range:
    #     :return:
    #     '''
    #
    #     first_pose = np.zeros(2)
    #
    #     res = minimize(self.standart_cost_func,
    #                    first_pose,
    #                    method='L-BFGS-B',
    #                    jac = False)
    #     self.history_pose[1,:] = first_pose
    #     if res.fun < 0.5:
    #         for i in range(self.sample_vector.shape[0]):
    #             for j in range(first_pose.shape[0]):
    #                 self.sample_vector[i,j] = first_pose[j] + np.random.normal(0.0,self.pose_var)
    #             for j in range(beacon_range.shape[0]):
    #                 self.sample_vector[i,first_pose.shape[0]+j] = beacon_range[j] + np.random.normal(0.0,self.beacon_var)
    #
    #     else:
    #         for i in range(self.sample_vector.shape[0]):
    #             for j in range(first_pose.shape[0]):
    #                 self.sample_vector[i,j] = first_pose[j] + np.random.normal(0.0,self.pose_var*2)
    #             for j in range(beacon_range.shape[0]):
    #                 self.sample_vector[i,first_pose.shape[0]+j] = beacon_range[j] + np.random.normal(0.0,self.beacon_var*2)



    def StateEqu(self):
        '''

        :return:
        '''

    def standart_cost_func(self, pose):
        '''

        :param pose:
        :return:
        '''
        pose_3d = np.zeros(3)

        pose_3d[0:2] = pose
        pose_3d[2] = self.z_offset

        if self.beaconPose.shape[0] == 0:
            print("ERROR: must setBeaconPose first")

        dis = np.ones(self.beaconPose.shape[0])
        for i in range(self.beaconPose.shape[0]):
            dis[i] = np.linalg.norm(pose_3d - self.beaconPose[i, :])

        return np.linalg.norm(dis - self.currentRange)

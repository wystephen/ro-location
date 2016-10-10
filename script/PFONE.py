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
        self.score = np.zeros_like(self.weight_vector)

        self.position_num = position_num
        self.beacon_num = beacon_num

        self.ign = 100

        # self.history_pose = np.zeros([2, position_num])
        # self.history_range = np.zeros([2, beacon_num])
        #
        # for i in range(position_num):
        #     self.history_pose[0, i] = np.random.normal(0.0, 1.0)
        # for i in range(beacon_num):
        #     self.history_range[0, i] = np.random.normal(0.0, 1.0)

    def reset(self, position_num,
              beacon_num,
              particle_num):
        '''

        :param position_num:
        :param beacon_num:
        :param particle_num:
        :return:
        '''

        self.sample_vector = np.zeros([particle_num, position_num + beacon_num])
        self.weight_vector = np.ones([particle_num, 1])
        self.score = np.zeros_like(self.weight_vector)

        self.position_num = position_num
        self.beacon_num = beacon_num


    def setPFParameter(self, pose_var=0.5, beacon_var=0.5, z_offset=1.12):
        '''

        :param pose_var:
        :param beacon_var:
        :param z_offset:
        :return:
        '''
        self.pose_var = pose_var
        self.beacon_var = beacon_var
        self.z_offset = z_offset

        self.state_var = np.zeros_like(self.sample_vector[0, :])
        for i in range(self.state_var.shape[0]):
            if i < self.position_num:
                self.state_var[i] = self.pose_var
            else:
                self.state_var[i] = self.beacon_var


    def setBeaconPose(self, beaconpose):
        '''
        Pose of beacons.

        :param beaconpose:
        :return:
        '''
        self.beaconPose = beaconpose
        self.currentRange = np.zeros(beaconpose.shape[0])

        return True

    def InitialValue(self, beacon_range):
        '''

        :param beacon_range:
        :return:
        '''

        first_pose = np.zeros(2)
        self.currentRange = beacon_range

        res = minimize(self.standart_cost_func,
                       first_pose,
                       method='L-BFGS-B',
                       jac=False)
        for i in range(self.sample_vector.shape[0]):
            self.sample_vector[i, 0] = first_pose[0] + np.random.normal(0.0, 0.05)
            self.sample_vector[i, 1] = first_pose[1] + np.random.normal(0.0, 0.05)
        # self.history_pose[1,:] = first_pose
        # if res.fun < 0.5:
        #     for i in range(self.sample_vector.shape[0]):
        #         for j in range(first_pose.shape[0]):
        #             self.sample_vector[i,j] = first_pose[j] + np.random.normal(0.0,self.pose_var)
        #         for j in range(beacon_range.shape[0]):
        #             self.sample_vector[i,first_pose.shape[0]+j] = beacon_range[j] + np.random.normal(0.0,self.beacon_var)
        #
        # else:
        #     for i in range(self.sample_vector.shape[0]):
        #         for j in range(first_pose.shape[0]):
        #             self.sample_vector[i,j] = first_pose[j] + np.random.normal(0.0,self.pose_var*2)
        #         for j in range(beacon_range.shape[0]):
        #             self.sample_vector[i,first_pose.shape[0]+j] = beacon_range[j] + np.random.normal(0.0,self.beacon_var*2)

    def ReSample(self):
        '''

        :return:
        '''

        # normlized
        # print(np.linalg.norm(self.weight_vector))
        self.weight_vector = self.weight_vector / np.sum(self.weight_vector)

        beta = np.zeros_like(self.weight_vector)
        for i in range(self.weight_vector.shape[0]):
            if i == 0:
                beta[i] = self.weight_vector[i]
            else:
                beta[i] = beta[i - 1] + self.weight_vector[i]

        # TODO:Check the beta[last_index] is it similar to 1

        tmp_sample_vector = np.zeros_like(self.sample_vector)
        tmp_weight_vector = np.zeros_like(self.weight_vector)
        for i in range(tmp_sample_vector.shape[0]):
            rnd = np.random.uniform()
            # print(rnd)

            for j in range(beta.shape[0]):
                if rnd < beta[j]:
                    # print("EE!")
                    tmp_sample_vector[i, :] = self.sample_vector[j, :]
                    tmp_weight_vector[i, :] = self.weight_vector[j, :]
                elif j == beta.shape[0] - 1:
                    # print("EE2")
                    tmp_sample_vector[i, :] = self.sample_vector[j, :]
                    tmp_weight_vector[i, :] = self.weight_vector[j, :]
                else:
                    # print("ERROR")
                    tmp_sample_vector[i, :] = self.sample_vector[j, :]
                    tmp_weight_vector[i, :] = self.weight_vector[j, :]
        self.weight_vector = tmp_weight_vector
        self.sample_vector = tmp_sample_vector

    def StateEqu(self, delta_sample_vec, the_current_range):
        '''

        :return:
        '''

        # for i in range(self.sample_vector.shape[0]):
        #     for j in range(self.sample_vector.shape[1]):
        #         self.sample_vector[i, j] += np.random.normal(delta_sample_vec[j] , self.state_var[j])
                # self.sample_vector[i, j] += np.random.normal(0.0, self.state_var[j])

        self.currentRange = the_current_range
        for i in range(self.sample_vector.shape[0]):
            self.sample_vector[i, 0:2] = self.get_pose(self.sample_vector[i, 0:2])
            # + \
            # [np.random.normal(0.0,self.state_var[0] / 10.0),
            #  np.random.normal(0.0,self.state_var[1]/10.0)]
        # self.sample_vector[:,0:2] += self.get_pose(self.sample_vector[0:2])
        self.sample_vector[:, 0:2] += np.random.normal(0.0, self.state_var[0] / 10.0,
                                                       size=(self.sample_vector.shape[0], 2))

    def GetResult(self):
        '''

        :return:
        '''

        # normlized
        # print(np.linalg.norm(self.weight_vector))
        self.weight_vector = self.weight_vector / np.sum(self.weight_vector)

        result = np.zeros_like(self.sample_vector[0, :])
        # print("norm weight ",np.linalg.norm(self.weight_vector))
        # TODO: USE NUMPY BOADCAST TO SPEED UP THIS STEP
        # for i in range(self.sample_vector.shape[0]):
        #     result += self.sample_vector[i, :] * self.weight_vector[i]
        result = np.sum(self.sample_vector * self.weight_vector, axis=0)

        # print("std:",np.std(self.sample_vector[:,0]),np.std(self.sample_vector[:,1]))

        # max_index = np.argmax(self.weight_vector)
        # result = self.sample_vector[max_index,:]

        return result

    def ObserveEva(self, all_range):
        '''

        :param all_range:
        :return:
        '''

        for i in range(self.weight_vector.shape[0]):
            # self.score[i] = self.GetScore(self.sample_vector[i, :], all_range)
            self.score[i] = self.GetComplexScore(self.sample_vector[i, :], all_range)
            self.weight_vector[i] = (self.weight_vector[i]) * (self.score[i])



    def GetScore(self, state_vec, all_range):
        '''

        :param state_vec:
        :param all_range:
        :return:
        '''
        pose = np.zeros(3)
        pose[2] = self.z_offset
        pose[0:2] = state_vec[0:self.position_num]
        the_range = state_vec[self.position_num:]

        dis = np.zeros_like(the_range)
        for i in range(the_range.shape[0]):
            dis[i] = np.linalg.norm(pose - self.beaconPose[i, :])

        return 1 / np.linalg.norm(dis - all_range)

    def GetComplexScore(self, state_vec, all_range):
        '''
        all range
        :param state_vec:
        :param all_range:
        :return:
        '''

        self.currentRange = all_range

        cost = self.standart_cost_func(state_vec[0:2])
        if cost == 1000:
            cost = 3.0 / cost
        else:
            cost_vector = np.zeros(4)
            for i in range(4):
                self.ign = i
                cost_vector[i] = self.cost_func(state_vec[0:2])
                if i < 4:
                    cost_vector[i] = 2.0 / (cost_vector[i] + 0.000001)
                else:
                    cost_vector[i] = 3.0 / (cost_vector[i] + 0.000001)

            cost = np.max(cost_vector)
            # cost = np.mean(cost_vector)

        return cost



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
            if i == self.ign:
                dis[i] = self.currentRange[i]

        return np.linalg.norm(dis - self.currentRange)

    def cost_func(self, pose):
        dis_err = np.zeros(3)

        t_pose = np.zeros(3)
        t_pose[0:2] = pose
        t_pose[2] = 1.12
        tmp_sum = np.sum(self.currentRange)

        for i in range(self.beaconPose.shape[0]):

            dis_err[i] = (np.linalg.norm(t_pose - self.beaconPose[i, :]) - self.currentRange[i]) / np.sqrt(
                self.currentRange[i] + 0.01)

            if self.ign > self.beaconPose.shape[0]:
                if dis_err[i] < 0.0:
                    dis_err[i] *= 0.5
                else:
                    dis_err[i] *= 1.0

            if i == self.ign:
                dis_err[i] = 0.0
                tmp_sum -= self.currentRange[i]
                # TODO!!!! check normalize parameter in this equation.
        return np.linalg.norm(dis_err) * tmp_sum

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
        if tmp_pose.fun < -1.0:
            re_pose = tmp_pose.x[0:2]
            print("ERROR : THIS FORK SHOUDN'T BE RUN.")
            # print(tmp_pose.fun)
        else:
            mul_re = np.zeros([3, 3])
            for i in range(3):
                self.ign = i

                dis_range = 0.3
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

            re_pose = mul_re[min_index, 0:2]
            # print(mul_re[min_index,2])

            return re_pose

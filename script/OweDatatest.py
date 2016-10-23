# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 23　10:37

import numpy as np

import scipy as sp

import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from scipy.optimize import minimize

from log_process import seq_process

class triangle3d:
    def __init__(self):
        self.beaconset = np.ones([3,3])
        self.range = np.ones(4)

    def costfunction(self,pose):
        dis_err = self.beaconset-pose
        dis_err = dis_err ** 2.0
        dis_err = np.sum(dis_err,axis=1)
        dis_err = dis_err ** 0.5
        err = dis_err - self.range
        return np.linalg.norm(err)


    def GetPose(self,beaconset,ranges):
        '''

        :param range:
        :return:
        '''
        self.beaconset = beaconset
        self.range = ranges
        # print(self.beaconset.shape,self.range.shape)
        res = minimize(self.costfunction,
                            [0.0,0.0,3.0],
                            method='L-BFGS-B',
                       bounds=((-100, 100),
                               (-100, 100),
                               (0.0,10.0)),
                            jac=False)
        print(res.fun,res.x)
        return res.x





if __name__ == '__main__':
    se = seq_process()
    # se.process_file(file_name='LOGBig/LOG_2016_10_19_16_1_18.data')
    se.process_file(file_name='LOGBig/LOG_2016_10_19_16_3_20.data')
    # se.process_file(file_name='LOGBig/LOG_2016_10_19_16_6_14.data')
    beacon_set = np.loadtxt("./LOGBig/beaconset")

    ###############################################
    # se.process_file("20161019log/LOG_2016_10_19_10_23_24.data")
    # se.process_file("20161019log/LOG_2016_10_19_10_42_28.data")
    # se.process_file("20161019log/LOG_2016_10_19_10_31_0.data")
    #
    # beacon_set = np.loadtxt("./20161019log/beaconset")


    '''
    Compute the range of
    '''

    print(beacon_set)

    data_range = np.loadtxt("atrange.txt")
    data_range /= 1000.0

    # print(data_range.mean(axis=1))

    t3d = triangle3d()

    result_x = np.zeros([data_range.shape[0],3])

    for i in range(data_range.shape[0]):
        result_x[i,:] = t3d.GetPose(beacon_set,data_range[i,:])
        if i % 10 == 0:
            print (i*1.0/data_range.shape[0])



###############
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.plot(result_x[:,0],result_x[:,1],result_x[:,2],'r+-')
    ax.legend()
###########
    # plt.subplot(2,1,1)
    # # plt.plot(result_x[:,0],result_x[:,1],result_x[:,2],'r-+')
    # plt.plot(result_x[:,0],result_x[:,1],'r-+')
    # plt.subplot(2,1,2)
    # plt.plot(result_x[:,2],'r-+')
##############

    plt.grid(True)
    plt.show()


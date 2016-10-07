# -*- coding:utf-8 -*-
# Create by steve in 16-10-7 at 上午10:09

import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

import filter_fram

if __name__ == '__main__':
    '''
    Some config data.
    '''
    beacon_001_ID = 1

    beacon_001_X = 0.21
    beacon_001_Y = -0.55
    beacon_001_Z = 1.94

    beacon_002_ID = 2

    beacon_002_X = 6.71
    beacon_002_Y = -0.33
    beacon_002_Z = 1.04

    beacon_003_ID = 3

    beacon_003_X = 6.7023
    beacon_003_Y = 4.3185
    beacon_003_Z = 1.33
    beacon = np.zeros([3, 3])


    beacon[0, 0] = beacon_001_X
    beacon[0, 1] = beacon_001_Y
    beacon[0, 2] = beacon_002_Z

    beacon[1, 0] = beacon_002_X
    beacon[1, 1] = beacon_002_Y
    beacon[1, 2] = beacon_002_Z

    beacon[2, 0] = beacon_003_X
    beacon[2, 1] = beacon_003_Y
    beacon[2, 2] = beacon_003_Z

    # gt should add odometry

    odometry_X = 2
    odometry_Y = 4
    odometry_PHI = 0.0  # deg
    odometry_offset = [odometry_X, odometry_Y, odometry_PHI]

    #Offset from robot to uwb sensor.
    robot_uwb_offset = [0.160, 0.000, 1.120]

    '''
    End config data
    '''

    gt = np.loadtxt("../datasets/uwb_ro-localization_demo_GT.txt")
    gt = gt[:, 1:4]

    #load beacon dataset
    beacon_info = np.loadtxt("../beacon_out.txt")
    # print(beacon_info)

    #load ground truth
    gt = gt[0:beacon_info.shape[0], :]


    #load filter result by cpp
    cpp_filter_out = np.loadtxt("../filter_out.txt")
    cpp_filter_out = cpp_filter_out[0:beacon_info.shape[0],:]


    #compute filter result
    sim_filter = filter_fram.filter()
    sim_filter.setInput(beacon_info,beacon)


    self_out = sim_filter.filter()

    #######################################
    # try to compute pose by self
    ######################################
    import triangle

    tg = triangle.triangle(beacon_info, beacon)
    tg.setRealvar(gt)

    tg_result = tg.localization()

    ######################################
    # end Trye to compoute pose by range
    ######################################

    #######################################################
    #Plot result
    #######################################################
    #error between ground truth and beacon location
    beacon_pose = beacon_info[:, 0:3]
    gt[:, 0] += odometry_offset[0]
    gt[:, 1] += odometry_offset[1]
    err_all = np.sum((beacon_pose[:, 0:2] - gt[:, 0:2]) ** 2.0, 1)
    err_all = err_all ** 0.5

    plt.figure(1)
    plt.plot(err_all)
    #error between ground truth and filter output
    plt.figure(2)
    err_filter = np.sum((cpp_filter_out[:,0:2]-gt[:,0:2]) ** 2.0 ,1)
    err_filter = err_filter ** 0.5

    plt.plot(err_filter)

    #error between ground truth and self filter

    plt.figure(3)
    err_self = np.sum((self_out[:,0:2]-gt[:,0:2])**2.0,1)
    err_self = err_self ** 0.5
    plt.plot(err_self)

    #
    #     b: blue
    # g: green
    # r: red
    # c: cyan
    # m: magenta
    # y: yellow
    # k: black
    # w: white

    plt.figure(4)

    plt.plot(gt[:,0],gt[:,1],'+r')
    plt.plot(beacon_info[:,0],beacon_info[:,1],'*b')

    plt.plot(cpp_filter_out[:,0],cpp_filter_out[:,1],'y.')

    plt.plot(self_out[:, 0], self_out[:, 1], '+c')

    plt.plot(tg_result[:, 0], tg_result[:, 1], '.k')

    plt.show()

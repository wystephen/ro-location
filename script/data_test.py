# -*- coding:utf-8 -*-
# Create by steve in 16-10-7 at 上午10:09

import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

if __name__ == '__main__':
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

    print(beacon)

    gt = np.loadtxt("../datasets/uwb_ro-localization_demo_GT.txt")
    gt = gt[:, 1:4]

    odometry_X = 2
    odometry_Y = 4
    odometry_PHI = 0.0  # deg
    odometry_offset = [odometry_X, odometry_Y, odometry_PHI]
    # gt should add odometry
    # print(gt)

    robot_uwb_offset = [0.160, 0.000, 1.120]
    # print(robot_uwb_offset)

    beacon_info = np.loadtxt("../beacon_out.txt")
    # print(beacon_info)

    gt = gt[0:beacon_info.shape[0], :]

    print(gt.shape)
    print(beacon_info.shape)

    beacon_pose = beacon_info[:, 0:3]
    gt[:, 0] += odometry_offset[0]
    gt[:, 1] += odometry_offset[1]
    err_all = np.sum((beacon_pose[:, 0:2] - gt[:, 0:2]) ** 2, 1)

    err_all = err_all ** 0.5
    print(err_all.shape)

    plt.figure(1)
    plt.plot(err_all)
    plt.show()

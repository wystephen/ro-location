# -*- coding:utf-8 -*-
# Create by steve in 16-10-18 at 下午6:21

import numpy as np

import re

import os

if __name__ == "__main__":
    dir_name = "../../cnn_data"

    data = np.zeros([337, 41 * 41])

    get_num_re = re.compile("\d{1,3}")

    for name in os.listdir(dir_name):
        index = get_num_re.findall(name)

        if not (int(index[0]) == 100) or True:
            td = np.loadtxt(dir_name + "/" + name)
            data[int(index[0]) - 101, :] = td.reshape([td.shape[0] * td.shape[1]])
            print(td.shape)
    print(data.min(axis=0))
    mxdmi = data.max() - data.min()
    min_v = data.min()
    data = (data - min_v) / mxdmi
    data = data - 0.5

    print(data.max(axis=0), data.min(axis=0))

    gt = np.loadtxt("../datasets/uwb_ro-localization_demo_GT.txt")
    gt = gt[:, 1:4]

    data_out = np.zeros([337, 20])
    print(gt.shape)
    for i in range(data_out.shape[0]):
        dx = gt[i + 1, 0] - gt[i, 0]
        dy = gt[i + 1, 1] - gt[i, 1]

        # ToDo:ssss How to do it?
        data_out[i, int((dx + 1.0) * 20.0)] = 1.0
        data_out[i, int((dy + 1.0) * 20.0)] = 1.0

    print(data_out)

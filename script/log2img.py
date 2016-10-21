# -*- coding:utf-8 -*-
# Create by steve in 16-10-14 at 下午3:50

import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

import scipy.misc


import os

import re

if __name__ == "__main__":
    print (os.listdir("../"))

    file_name_list = []

    for name in os.listdir("../../tmpdata"):
        if name[-1] == 't':
            file_name_list.append(name)
    # print(file_name_list)

    get_num_re = re.compile("\d{1,3}")


    for txt_file in file_name_list:
        # print(txt_file)
        index = get_num_re.findall(txt_file)
        if len(index) > 0:
            # print("../../tmpdata/" + txt_file)
            tmp_mat = np.loadtxt("../../tmpdata/" + txt_file)
            # print(tmp_mat.shape)
            #
            # print(index)

            index = int(index[0])

            tmp_mat = tmp_mat / tmp_mat.sum()

            scipy.misc.toimage(tmp_mat).save("../../tmpdata/" + str(index) + ".bmp")

        # plt.imshow(tmp_mat)
        # plt.show()




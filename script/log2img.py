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

    for name in os.listdir("../"):
        if name[-1] == 't':
            file_name_list.append(name)
    print(file_name_list)

    get_num_re = re.compile(r"\d{1,3}")


    for txt_file in file_name_list:
        tmp_mat = np.loadtxt("../"+txt_file)

        index = get_num_re.findall(txt_file)

        index = int(index[0])


        scipy.misc.toimage(tmp_mat).save("./"+str(index)+".bmp")

        # plt.imshow(tmp_mat)
        # plt.show()




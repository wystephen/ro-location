# -*- coding:utf-8 -*-
# Create by steve in 16-10-30 at 下午10:35
import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

if __name__ == '__main__':
    '''
    transmit \n to \r\n in text files.
    '''

    print("frame")
    import os
    fa = open("LOG_2016_10_26_14_3_26.txt")
    tmp_log = open("log.data",'w')

    all_file = fa.readline()
    print(len(all_file))

    last_i = 0
    for i in range(len(all_file)):
        # print(i)
        if all_file[i] == '\\' and all_file[i+1] == 'n':
            tmp_log.write(all_file[last_i:i]+'\n')
            last_i   = i +2
            print("new line")
    tmp_log.close()

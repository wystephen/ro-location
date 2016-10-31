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
    file = open("LOG_2016_10_26_14_3_26.txt",'r')

    print(len(file.readlines()))

    

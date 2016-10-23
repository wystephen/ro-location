# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 23　10:37

import numpy as np

import scipy as sp

import matplotlib.pyplot as plt

from log_process import seq_process
if __name__ == '__main__':
    se = seq_process()
    se.process_file(file_name='LOGBig/LOG_2016_10_19_16_1_18.data')

    '''
    Compute the range of
    '''

    
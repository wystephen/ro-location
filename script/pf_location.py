# -*- coding:utf-8 -*-
# Create by steve in 16-10-7 at 上午10:10

import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

from filter_fram import filter_frame


class RangPf(filter_frame):
    def __init__(self):
        filter_frame.__init__(self)

    def initial_filter(self, N_praticles):
        '''
        Initial filter parameter and set some practicles.
        :param N_praticles:
        :return:
        '''

        return True

    def filter(self):
        '''
        main
        :return:
        '''

        return self.filter_result

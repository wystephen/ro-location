# -*- coding:utf-8 -*-
# Create by steve in 16-10-9 at 上午9:38

import numpy as np
import scipy as sp

from scipy.optimize import minimize


class PFONE:
    def __init__(self, position_num,
                 beacon_num,
                 particle_num
                 ):
        '''
        This function is only use to define some value for filter.
        :position_num
        :beacon_num
        :particle_num
        :return:
        '''
        self.sample_vector = np.zeros([particle_num, position_num + beacon_num])
        self.weight_vector = np.zeros([particle_num, 1])

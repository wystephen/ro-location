# -*- coding:utf-8 -*-
# Create by steve in 16-10-12 at 下午4:00
import numpy as np


class EKFONE:
    def __init__(self):
        self.x = 0.0
        self.P = 0.1
        self.H = 1.0
        self.I = 1.0
        self.K = 1.0
        self.R = 0.05
        self.Q = 0.01

        self.F = 1.0

    def Initial(self, x):
        self.x = x

    def filter(self, y):
        self.P = self.F * self.P * self.F + self.Q

        self.K = self.P * self.H / (self.H * self.P * self.H + self.R)

        self.x += self.K * (y - self.x)

        self.P = (self.I - self.K * self.H) * self.P

        # print(y,self.x,self.K)

        return self.x

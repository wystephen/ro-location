# -*- coding:utf-8 -*-
# Create by steve in 16-10-11 at 下午5:25

from theano import function, config, shared, sandbox
import theano.tensor as T
import numpy as np
import time
import theano

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
    beacon[0, 2] = beacon_001_Z

    beacon[1, 0] = beacon_002_X
    beacon[1, 1] = beacon_002_Y
    beacon[1, 2] = beacon_002_Z

    beacon[2, 0] = beacon_003_X
    beacon[2, 1] = beacon_003_Y
    beacon[2, 2] = beacon_003_Z

    real_pose = [1.98119, 4.07225, 1.12]
    all_range = [5.13628, 6.30111, 4.6493]

    pose = T.dvector('pose')
    gpu_range = T.dvector('gpu_range')
    gpu_set = T.dmatrix('gpu_set')
    cost = T.dscalar('cost')
    gc = T.dvector('gc')

    cost = T.sum((T.sum((pose - gpu_set) ** 2, axis=1) - gpu_range) ** 2)

    cost_f = theano.function(outputs=cost, inputs=[pose, gpu_range, gpu_set])

    gc = T.grad(cost, pose)

    all_fun = theano.function(outputs=gc, inputs=[pose, gpu_range, gpu_set])

    print("First val: ", cost_f(real_pose, all_range, beacon))

    z_pose = [2.0, 3.0, 1.0]
    for i in range(10000):
        grad_pose = all_fun(z_pose, all_range, beacon)
        f_val = cost_f(z_pose, all_range, beacon)
        z_pose = z_pose + 0.1 * grad_pose

        print(f_val)
        print(grad_pose, z_pose)

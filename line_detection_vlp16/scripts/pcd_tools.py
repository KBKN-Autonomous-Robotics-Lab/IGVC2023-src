#!/usr/bin/env python3

import numpy as np
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
import time
from sensor_msgs.msg import PointCloud2, PointField



def read_from_msg(msg: PointCloud2):
    pcd_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    # pcd_array: 1D array: [(x1,y1,z1,i1,?,?), (x2,y2,z2,i2,?,?), ...]
    X = np.zeros(len(pcd_array), dtype=np.float32)
    Y = np.zeros_like(X)
    Z = np.zeros_like(X)
    I = np.zeros_like(X)
    for i, p in enumerate(pcd_array):
        X[i], Y[i], Z[i], I[i] = p[0], p[1], p[2], p[3]
    
    return X, Y, Z, I



def create_pcd_msg(header, X, Y, Z, I):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
    ]
    points = np.zeros((X.shape[0], 4))
    for i in range(0, X.shape[0]):
        points[i, :] = X[i], Y[i], Z[i], I[i]


    return pc2.create_cloud(header, fields, points)



class PointsQueue:

    def __init__(self, queue_size):
        self.queue_size = queue_size
        self.points = np.zeros((4, queue_size), dtype=np.float32)
        self.pt = 0
        self.is_full = False
    
    def append(self, X, Y, Z, I):
        n = len(X)
        if (n >= self.queue_size):
            self.points[0, :] = X[:self.queue_size]
            self.points[1, :] = Y[:self.queue_size]
            self.points[2, :] = Z[:self.queue_size]
            self.points[3, :] = I[:self.queue_size]
            self.pt = 0
            self.is_full = True
        elif (self.pt+n > self.queue_size):
            end = self.queue_size - self.pt
            next_pt = n - end
            self.points[0, self.pt:] = X[:end]
            self.points[0, :next_pt] = X[end:]
            self.points[1, self.pt:] = Y[:end]
            self.points[1, :next_pt] = Y[end:]
            self.points[2, self.pt:] = Z[:end]
            self.points[2, :next_pt] = Z[end:]
            self.points[3, self.pt:] = I[:end]
            self.points[3, :next_pt] = I[end:]
            self.pt = next_pt
            self.is_full = True
        else:
            self.points[0, self.pt:self.pt+n] = X
            self.points[1, self.pt:self.pt+n] = Y
            self.points[2, self.pt:self.pt+n] = Z
            self.points[3, self.pt:self.pt+n] = I
            self.pt += n
            if (self.pt == 100): self.pt = 0
    
    def get_points(self):
        if self.is_full:
            return self.points[0,:], self.points[1,:], self.points[2,:], self.points[3,:]
        return self.points[0,:self.pt], self.points[1,:self.pt], self.points[2,:self.pt], self.points[2,:self.pt]
    
    def empty(self):
        self.pt = 0
        self.is_full = False
        



class TimeCounter:
    def __init__(self, count_time) -> None:
        self.counter = 0
        self.start_time = 0
        self.sum_time = 0
        self.count_time = count_time
    
    def start(self):
        self.start_time = time.perf_counter()
    
    def end(self):
        end_time = time.perf_counter()
        self.sum_time += (end_time - self.start_time)
        self.counter += 1
        if self.counter == self.count_time:
            t = self.sum_time/self.counter
            print(self.counter, "Times average:", '{:.5f}'.format(t), "(secs)")
            self.sum_time = 0
            self.counter = 0
            return t
    

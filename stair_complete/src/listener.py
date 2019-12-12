#!/usr/bin/env python

# use polygon as the message

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

import numpy as np
from sklearn.neighbors import NearestNeighbors


def poly2array(poly):
    p_num = len(poly.points)
    arr = np.zeros([p_num, 3])
    for i in range(p_num):
        arr[i] = poly.points[i].x, poly.points[i].y, poly.points[i].z
    return arr


def array2poly(arr):
    poly = Polygon()
    p_num = arr.shape[0]
    for i in range(p_num):
        point = Point32(arr[i, 0], arr[i, 1], arr[i, 2])
        poly.points.append(point)
    return poly


def knn_filter(samples, k=5, dist_threshold=0.05):
    print("oringinal gradient")
    print(samples.shape[0])
    print(samples)

    neigh = NearestNeighbors(k)
    neigh.fit(samples)
    dist, ind = neigh.kneighbors(return_distance=True)

    value_table = dist > dist_threshold
    index = np.nonzero(value_table)

    filtered_samples = np.delete(samples, index[0], axis=0)
    print(dist)
    print("filtered gradient")
    print(filtered_samples.shape[0])
    print(filtered_samples)
    return filtered_samples


class Filter:
    def __init__(self):
        rospy.init_node('filter_listener', anonymous=True)
        self.pub = rospy.Publisher("/grad_filtered", Polygon, queue_size=10)
        self.sub = rospy.Subscriber('/grad_to_filter', Polygon, self.lis_callback)
        return

    def listen(self):
        rospy.spin()
        return

    def lis_callback(self, poly):
        # rospy.loginfo(poly)

        grad_arr = poly2array(poly)

        filtered_grad_arr = knn_filter(grad_arr, 4, 0.05)

        poly_filterd = array2poly(filtered_grad_arr)

listener.py        self.pub.publish(poly_filterd)



        # print(filtered_grad_arr)
        return


if __name__ == '__main__':
    lis = Filter()
    while not rospy.is_shutdown():
        lis.listen()
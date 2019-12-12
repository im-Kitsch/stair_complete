#!/usr/bin/env python

# use marker as the message

import rospy

from geometry_msgs.msg import PoseArray

import numpy as np
from sklearn.neighbors import NearestNeighbors


def markers2arr(markers):
    # markers = PoseArray()
    p_num = len(markers.poses)
    arr = np.zeros([p_num, 7])
    for i in range(p_num):
        arr[i] = markers.poses[i].position.x, markers.poses[i].position.y, markers.poses[i].position.z,\
                 markers.poses[i].orientation.x, markers.poses[i].orientation.y,\
                 markers.poses[i].orientation.z, markers.poses[i].orientation.w
    return arr


def arr2markers(arr):

    return

class Filter:
    def __init__(self):
        rospy.init_node('filter_listener', anonymous=True)
        self.pub = rospy.Publisher("/visualizer/grad_pose", PoseArray, queue_size=10)
        self.sub = rospy.Subscriber('/grad_to_filter_marker', PoseArray, self.lis_callback)
        return

    def listen(self):
        rospy.spin()
        return

    def lis_callback(self, markers):
        # rospy.loginfo(markers)



        self.knn_filter(markers, 4, 0.05)

        # poly_filterd = array2poly(filtered_grad_arr)
        #
        # self.pub.publish(poly_filterd)

        # print(filtered_grad_arr)
        return

    def knn_filter(self, info_markers, k=5, dist_threshold=0.05):
        info_arr = markers2arr(info_markers)

        print("oringinal gradient")
        print(info_arr.shape[0])

        grads = info_arr[:, 3:6]

        neigh = NearestNeighbors(k)
        neigh.fit(grads)
        dist, ind = neigh.kneighbors(return_distance=True)

        value_table = dist > dist_threshold
        index = np.nonzero(value_table)

        filtered_samples = np.delete(info_arr, index[0], axis=0)
        print(dist)
        print("filtered gradient")
        print(filtered_samples.shape[0])
        print(filtered_samples)

        list_samples = np.array(info_markers.poses)
        filtered_poses = np.delete(list_samples, index[0])
        info_markers.poses = filtered_poses

        self.pub.publish(info_markers)
        self.pub.publish(info_markers)
        self.pub.publish(info_markers) 
        self.pub.publish(info_markers)

        return


if __name__ == '__main__':
    lis = Filter()
    while not rospy.is_shutdown():
        lis.listen()
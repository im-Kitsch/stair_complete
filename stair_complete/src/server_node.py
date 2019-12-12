#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server

from stair_complete.cfg import tutorialsConfig


def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("server_node", anonymous = False)

    srv = Server(tutorialsConfig, callback)
    rospy.spin()

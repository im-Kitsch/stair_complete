#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client


def callback(config):
    rospy.loginfo("Config set to {offset_x}".format(**config))


if __name__ == "__main__":

    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("offline_opt", timeout=30, config_callback=callback)

    r = rospy.Rate(.2)

    while not rospy.is_shutdown():


        result = client.update_configuration({"offset_x":0.05})

        r.sleep()
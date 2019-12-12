#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32


class Talker:
    def __init__(self):
        self.pub = rospy.Publisher("grad_to_filter", Polygon, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        return

    def talk(self, grad_arr=[]):

        poly = Polygon()
        point = Point32(1, 2, 3)
        poly.points.append(point)
        poly.points.append(point)
        rospy.loginfo(poly)
        self.pub.publish(poly)
        return


if __name__ == '__main__':
    talker = Talker()
    try:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            talker.talk()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

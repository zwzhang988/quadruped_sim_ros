#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def keyboard_publisher():
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('/keyboard_input', String, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        input_str = input("Enter input: ")
        rospy.loginfo("Publishing: %s", input_str)
        pub.publish(input_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass

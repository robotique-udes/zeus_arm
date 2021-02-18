#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('motor_cmd', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    cmd = 10.5
    pub.publish(cmd)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
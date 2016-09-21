#!/usr/bin/env python
# license removed for brevity
import rospy, time
from std_msgs.msg import String

def talker():
    #pub = rospy.Publisher('psoc_tx', String)#, queue_size=1000)
    pub = rospy.Publisher('drive_tx', String)#, queue_size=1000)
    rospy.init_node('talker', anonymous=True)
    
    '''rate = rospy.Rate(1) #1Hz
    msg = "a\r"
    while not rospy.is_shutdown():
	    pub.publish(msg)
	    sleep(5)
	    msg = "F 100\r"
	    pub.publish(msg)
	    sleep(3)
	    msg = "F 0\r"
	    pub.publish(msg)
    	    rate.sleep()'''
    rate = rospy.Rate(10) # 10hz
    msg = "F 100\r"
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo("%s", msg);
	sleep(3)
        msg = "F 0\r"
	'''sleep(5)
	msg = "F 0\r"
	sleep(4)
	msg = ""'''
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

import rospy
from std_msgs.msg import Int32MultiArray

def talker():
    pub = rospy.Publisher('Batterij', Int32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = [10, 15]
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
        exit()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
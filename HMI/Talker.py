import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('Batterij', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "[10, 15]"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        exit()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
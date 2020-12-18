import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('Chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "Hello World"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        exit()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
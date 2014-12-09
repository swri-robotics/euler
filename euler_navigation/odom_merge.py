#!/usr/bin/python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy

class OdomMerger:
    def __init__(self):
        rospy.init_node('odom_merger')
        self.sub1 = rospy.Subscriber('/euler/odom', Odometry, self.odom_cb)
        self.sub2 = rospy.Subscriber('/euler/cmd_vel', Twist, self.twist_cb)
        self.pub = rospy.Publisher('/euler/odom_smooth', Odometry)
        self.twist = Twist()
        
    def twist_cb(self, msg):
        self.twist = msg
        
    def odom_cb(self, msg):
        msg.twist.twist = self.twist
        self.pub.publish(msg)
        
if __name__=='__main__':
    om = OdomMerger()
    rospy.spin()
        

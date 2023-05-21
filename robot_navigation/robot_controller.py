'''python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2

class Robot:
    def __init__(self):
        rospy.init_node('robot_controller')
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = 5.0
        self.target_y = 5.0
        self.target_yaw = 0.0
        self.vel_msg = Twist()

    def laser_callback(self, msg):
        # process laser scan data
        pass

    def odom_callback(self, msg):
        # process odometry data
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw

    def move_to_target(self):
        # calculate target angle and distance
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        target_yaw = atan2(dy, dx)
        target_distance = (dx ** 2 + dy ** 2) ** 0.5

        # calculate angular velocity
        angular_error = target_yaw - self.current_yaw
        if angular_error > 3.14159:
            angular_error -= 2 * 3.14159
        elif angular_error < -3.14159:
            angular_error += 2 * 3.14159
        angular_velocity = 0.5 * angular_error

        # calculate linear velocity
        linear_velocity = 0.5 * target_distance

        # set velocity message
        self.vel_msg.linear.x = linear_velocity
        self.vel_msg.angular.z = angular_velocity
        self.pub.publish(self.vel_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.move_to_target()
            self.rate.sleep()

if __name__ == '__main__':
    robot = Robot()
    robot.run()

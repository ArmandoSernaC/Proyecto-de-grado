#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TangentBugNode:
    def __init__(self):
        rospy.init_node('tangent_bug_node', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.callbackLaser)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callbackPose)
        self.rate = rospy.Rate(10)
        self.robot_position = None
        self.robot_rotation = None
        self.dreach = float('inf')
        self.dfollowed = float('inf')
        self.Oi = None

    def callbackPose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_rotation = yaw
    
    def callbackLaser(self, msg):
        if self.robot_position is not None and self.robot_rotation is not None:
            # Trata los datos del sensor láser
            ranges = msg.ranges
            laser_data = []

            for i, distance in enumerate(ranges):
                if not math.isinf(distance):
                    angle = self.robot_rotation + msg.angle_min + i * msg.angle_increment
                    x = self.robot_position[0] + distance * math.cos(angle)
                    y = self.robot_position[1] + distance * math.sin(angle)
                    laser_data.append((x, y, distance))

            # Llamar a los métodos correspondientes para procesar los datos del láser
            self.mtg_distances(laser_data)
            self.bf_distances(laser_data)

    def move_forward(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.2  # Velocidad lineal hacia adelante
        self.velocity_publisher.publish(vel_msg)

    def turn_left(self):
        vel_msg = Twist()
        vel_msg.angular.z = 0.2  # Velocidad angular hacia la izquierda
        self.velocity_publisher.publish(vel_msg)

    def motion_to_goal(self):
        # ...
        pass

    def mtg_distances(self):
        # ...
        pass

    def boundary_following(self):
        # ...
        pass

    def bf_distances(self):
        # ...
        pass

    def transform_angle(self, waypoint):
        # ...
        pass

    def intersection(self, P1, P2, C):
        # ...
        pass

    def inflate_obstacles(self, P1, P2):
        # ...
        pass

    def move_target(self, Oi):
        # ...
        pass

    def blocking_obs(self):
        # ...
        pass

    def point_is_between(self, A, B, C):
        # ...
        pass

    def distance(self, point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def execute(self):
        while not rospy.is_shutdown():
            if self.robot_position is not None and self.robot_rotation is not None:
                # Implementa el algoritmo Tangent Bug aquí
                # Llama a los métodos según la lógica del algoritmo
                self.rate.sleep()

if __name__ == '__main__':
    try:
        tangent_bug_node = TangentBugNode()
        tangent_bug_node.execute()
    except rospy.ROSInterruptException:
        pass

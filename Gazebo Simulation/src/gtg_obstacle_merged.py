#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class GoalToGoal:
    def __init__(self, initial_x, initial_y, goal_x, goal_y):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False

        self.pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/atom/odom', Odometry, self.odom_callback)

        self.initial_x = initial_x
        self.initial_y = initial_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.linear_speed = 0.2
        self.angular_speed = 0.3

        self.obstacle_detected = False
        self.closest_obstacle_angle = 0.0
        self.closest_obstacle_distance = 0.0
        self.obstacle_distances = []

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, self.current_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                         msg.pose.pose.orientation.y,
                                                         msg.pose.pose.orientation.z,
                                                         msg.pose.pose.orientation.w])

    def scan_callback(self, msg):
        self.obstacle_distances = msg.ranges

        # Check if any distance measurement is less than the threshold based on specified ranges
        if -1.57 <= self.closest_obstacle_angle <= -1.0 or 1.0 <= self.closest_obstacle_angle <= 1.57:
            self.obstacle_detected = any(0.0 < dist < 0.5 for dist in self.obstacle_distances)
        else:
            self.obstacle_detected = any(0.0 < dist < 0.7 for dist in self.obstacle_distances)

        if self.obstacle_detected:
            closest_obstacle_index = self.obstacle_distances.index(min(self.obstacle_distances))
            self.closest_obstacle_angle = msg.angle_min + closest_obstacle_index * msg.angle_increment
            self.closest_obstacle_distance = self.obstacle_distances[closest_obstacle_index]
        else:
            self.closest_obstacle_angle = 0.0
            self.closest_obstacle_distance = 0.0

    def orient_towards_goal(self, target_x, target_y):
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)

        while not rospy.is_shutdown():
            if not self.obstacle_detected:
                # Align the robot's orientation with the goal direction
                angle_diff = target_angle - abs(self.current_yaw)
                if abs(angle_diff) > 0.01:
                    twist_msg = Twist()
                    twist_msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                    self.pub.publish(twist_msg)
                    rospy.sleep(0.1)
                else:
                    break
            else:
                # Obstacle detected, avoid the obstacle
                self.obstacle_avoidance()

        # Stop rotating
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        print(twist_msg)
        print('otg')

    def move_to_goal(self, target_x, target_y):
        # Orient towards the goal
        self.orient_towards_goal(target_x, target_y)

        while not rospy.is_shutdown():
            if not self.obstacle_detected:
                # No obstacle, move forward to the goal
                twist_msg = Twist()
                twist_msg.linear.x = self.linear_speed
                self.pub.publish(twist_msg)
                rospy.sleep(0.1)

                # Check if the goal is reached
                distance_to_goal = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
                if distance_to_goal < 0.2:
                    rospy.loginfo("Goal reached. Stopping goal-to-goal navigation.")
                    self.goal_reached = True
                    break
            else:
                # Obstacle detected, avoid the obstacle
                self.obstacle_avoidance()

        # Stop moving
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        print(twist_msg)
        print('mtg')
        rospy.sleep(1.0)

    def obstacle_avoidance(self):
        left_obstacle = any(dist < 1.0 for dist in self.obstacle_distances[:len(self.obstacle_distances)//2])
        right_obstacle = any(dist < 1.0 for dist in self.obstacle_distances[len(self.obstacle_distances)//2:])

        if left_obstacle and right_obstacle:
            # Obstacles on both sides, take reverse while turning
            twist_msg = Twist()
            twist_msg.linear.x = -0.5  # Reverse
            twist_msg.angular.z = 1.0 if self.closest_obstacle_angle < 0 else -1.0  # Turn while reversing
        
        else:
            # Obstacle on one side, turn away from the obstacle
            angle_diff = self.closest_obstacle_angle
            twist_msg = Twist()
            twist_msg.linear.x = 0.2  # Stop linear motion
            twist_msg.angular.z = 0.5 if angle_diff < 0 else -0.5  # Turn away from the obstacle

        self.pub.publish(twist_msg)
        rospy.sleep(1.0)
        print(twist_msg)
        print('oA')
        rospy.sleep(1.0)

        if not self.obstacle_detected:
            rospy.loginfo("No obstacle detected. Transitioning to move_to_goal.")
            self.move_to_goal(self.goal_x, self.goal_y)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_and_goal_to_goal', anonymous=True)

    initial_x = float(input("Enter initial x-coordinate: "))
    initial_y = float(input("Enter initial y-coordinate: "))
    goal_x = float(input("Enter goal x-coordinate: "))
    goal_y = float(input("Enter goal y-coordinate: "))

    goal_to_goal = GoalToGoal(initial_x, initial_y, goal_x, goal_y)

    # Move to initial coordinates
    goal_to_goal.move_to_goal(initial_x, initial_y)

    # Check if obstacle is detected during orientation to the goal
    if goal_to_goal.obstacle_detected:
        goal_to_goal.obstacle_avoidance()

    # Move forward to the goal
    goal_to_goal.move_to_goal(goal_x, goal_y)


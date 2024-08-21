#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class GoalToGoal:
    def __init__(self, initial_x, initial_y, goal_x, goal_y):
        print('init')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False
        self.target_x = 0
        self.target_y = 0
        self.final_position_reached = False

        self.pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=0)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/atom/odom', Odometry, self.odom_callback)

        self.initial_x = initial_x
        self.initial_y = initial_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.linear_speed = 0.5
        self.angular_speed = 0.4

        self.obstacle_detected = False
        self.closest_obstacle_angle = 0.0
        self.closest_obstacle_distance = 0.0
        self.obstacle_distances = []

        self.generated_points = [(initial_x, initial_y)]  # Add the initial point
        self.generate_all_points()  # Call the function to generate points
    
    def get_yaw(orientation):
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw
    
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

    def orient_towards_goal(self):
        print('otg')

        while not rospy.is_shutdown():
            target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
            # Align the robot's orientation with the goal direction
            if (-3.15 < self.current_yaw < -1.4 or 1.4 < self.current_yaw < 3.15) and (-3.15 < target_angle < -1.4 or 1.4 < target_angle < 3.15) and (target_angle * self.current_yaw < 0):
                if target_angle*self.current_yaw > 0:
                    angle_diff = target_angle - self.current_yaw
                if target_angle*self.current_yaw < 0  and self.current_yaw > target_angle:
                    angle_diff = self.current_yaw
                elif target_angle*self.current_yaw < 0 and self.current_yaw < target_angle:
                    angle_diff = self.current_yaw
            else:
                angle_diff = target_angle - self.current_yaw
            if abs(angle_diff) > 0.01:
                twist_msg = Twist()
                twist_msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                self.pub.publish(twist_msg)
            else:
                break

        # Stop rotating
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.sleep(0.5)

    def move_to_goal(self, target_x, target_y):
        print('mtg')
        # Update the target position in case it changes
        self.target_x = target_x
        self.target_y = target_y
        self.orient_towards_goal()

        while not rospy.is_shutdown():
            if not self.obstacle_detected:
                # No obstacle, move forward to the goal
                twist_msg = Twist()
                twist_msg.linear.x = self.linear_speed
                self.pub.publish(twist_msg)
                rospy.sleep(0.1)

                # Check if the goal is reached
                distance_to_goal = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
                if distance_to_goal <= 0.2:
                    print('here')
                    rospy.loginfo("Goal reached. Stopping goal-to-goal navigation.")
                    print('overhere')
                    self.goal_reached = True
                    break
            else:
                # Obstacle detected, avoid the obstacle
                self.obstacle_avoidance()    

        # Stop moving
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.sleep(0.5)

    def obstacle_avoidance(self):
        print('oA')
        left_obstacle = any(dist < 1.0 for dist in self.obstacle_distances[:len(self.obstacle_distances)//2])
        right_obstacle = any(dist < 1.0 for dist in self.obstacle_distances[len(self.obstacle_distances)//2:])

        if left_obstacle and right_obstacle:
            # Obstacles on both sides, take reverse while turning
            twist_msg = Twist()
            twist_msg.linear.x = -0.5  # Reverse
            twist_msg.angular.z = 2.0 if self.closest_obstacle_angle < 0 else -1.0  # Turn while reversing
        
        else:
            # Obstacle on one side, turn away from the obstacle
            angle_diff = self.closest_obstacle_angle
            twist_msg = Twist()
            twist_msg.linear.x = 0.2  # Stop linear motion
            twist_msg.angular.z = 0.5 if angle_diff < 0 else -0.5  # Turn away from the obstacle

        self.pub.publish(twist_msg)
        print(twist_msg)
        rospy.sleep(1)

        if not self.obstacle_detected:
            rospy.loginfo("No obstacle detected. Transitioning to move_to_goal.")
            self.move_to_goal(x_after_obstacle, y_after_obstacle)

    def generate_all_points(self):
        print('gap')
        x, y = self.initial_x, self.initial_y
        step_size = self.goal_x - self.initial_x
        t = abs(self.goal_y - self.initial_y)
        t = int(t)
        if t % 2 == 0:
            loop_iterate = t + 1
        else:
            loop_iterate = t
        for i in range(loop_iterate):

            if i % 2 == 0:
                next_point = (x + step_size, y)
            else:
                next_point = (x, y)
            print(next_point)
            self.generated_points.append(next_point)

            if x + step_size == self.goal_x and y == self.goal_y:
                print('Goal reached. Stopping point generation.')
                break

            if step_size < 0:
                y = y - 1
            else:
                y += 1
            if i % 2 == 0:
                next_point = (x + step_size, y)
            else:
                next_point = (x, y)
            print(next_point)
            self.generated_points.append(next_point)

            if x == self.goal_x and y == self.goal_y:
                print('Goal reached. Stopping point generation.')
                break

    def travel_zigzag(self):
        print('tz')

        for i in range(len(self.generated_points)):
            global x_after_obstacle, y_after_obstacle
            x_after_obstacle = self.generated_points[i][0]
            y_after_obstacle = self.generated_points[i][1]
            self.move_to_goal(self.generated_points[i][0], self.generated_points[i][1])
            rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('goal_to_goal', anonymous=True)

    initial_x = float(input("Enter initial x-coordinate: "))
    initial_y = float(input("Enter initial y-coordinate: "))
    goal_x = float(input("Enter goal x-coordinate: "))
    goal_y = float(input("Enter goal y-coordinate: "))

    goal_to_goal = GoalToGoal(initial_x, initial_y, goal_x, goal_y)

    # Access the generated points
    print("Generated Points:", goal_to_goal.generated_points)

    # Move forward to the goal
    goal_to_goal.travel_zigzag()

#!/usr/bin/env python

import rospy
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
        self.initial_position_reached = False  # Flag to track initial position reach
        self.final_position_reached = False

        self.pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/atom/odom', Odometry, self.odom_callback)

        self.initial_x = initial_x
        self.initial_y = initial_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.linear_speed = 0.5
        self.angular_speed = 0.4

        self.move_to_initial_goal(self.initial_x, self.initial_y)
        self.initial_position_reached = True

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, self.current_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                         msg.pose.pose.orientation.y,
                                                         msg.pose.pose.orientation.z,
                                                         msg.pose.pose.orientation.w])

    def orient_towards_goal(self):
        print('otg')
        target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)

        while not rospy.is_shutdown():
            # Align the robot's orientation with the goal direction
            angle_diff = target_angle - abs(self.current_yaw)
            # print('angles:')
            # print(angle_diff)
            # print(target_angle)
            # print(self.current_yaw)
            # print(' ')
            if abs(angle_diff) > 0.01:
                twist_msg = Twist()
                twist_msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                self.pub.publish(twist_msg)
                rospy.sleep(0.1)
            else:
                break

        # Stop rotating
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.sleep(1.0)

    def move_to_initial_goal(self, target_x, target_y):
        print('mtg')
        self.target_x = target_x
        self.target_y = target_y

        self.orient_towards_goal()

        while not rospy.is_shutdown():
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

        # Stop moving
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.sleep(1.0)

    def move_to_goal(self):
        print('mtg')
        while not rospy.is_shutdown():
            # No obstacle, move forward to the goal
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed
            self.pub.publish(twist_msg)
            rospy.sleep(0.1)

            # Check if the goal is reached
            distance_to_goal = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
            if distance_to_goal < 0.2:
                rospy.loginfo("Goal reached. Stopping goal-to-goal navigation.")
                self.goal_reached = True
                break

        # Stop moving
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.sleep(1.0)

    def generate_next_point(self, current_point):
        print('gnp')
        x, y = current_point
        step_size = goal_x - initial_x  # Adjust this based on the distance between points along the primary axis
        print(step_size)
        t = goal_y - initial_y
        t = int(t)
        if t%2 == 0:
            loop_iterate = t*2 + 1
        else:
            loop_iterate = t*2

        for i in range(loop_iterate):
            if self.final_position_reached == False:
                distance_to_final_goal_1 = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
                print(distance_to_final_goal_1)
                if distance_to_final_goal_1 >= 0.2:
                    if i % 2 == 0:
                        next_point = (x + step_size, y)
                    else:
                        next_point = (x, y)
                    print(next_point)
                    self.target_x, self.target_y = next_point
                    self.orient_towards_goal()

                    # Move to the next point
                    self.move_to_goal()

                else:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    self.pub.publish(twist_msg)
                    rospy.loginfo("Goal reached.")
                    self.final_position_reached = True
                    break

                distance_to_final_goal_2 = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
                print(distance_to_final_goal_2)
                if distance_to_final_goal_2 >= 0.2:
                    if step_size < 0:
                        y = y - 1
                    else:
                        y += 1
                    if i % 2 == 0:
                        next_point = (x + step_size, y)
                    else:
                        next_point = (x, y)
                    print(next_point)
                    self.target_x, self.target_y = next_point
                    self.orient_towards_goal()
                    self.move_to_goal()

            else:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                self.pub.publish(twist_msg)
                rospy.loginfo("Goal reached.")
                self.final_position_reached = True
                break


    def travel_zigzag(self):
        print('tz')
        
        while not rospy.is_shutdown():
            current_point = (self.current_x, self.current_y)
            # Generate points and traverse to the final goal 
            self.generate_next_point(current_point)

            # Move to the final goal
            self.move_to_goal()
    
    print(' ')

if __name__ == '__main__':
    rospy.init_node('goal_to_goal', anonymous=True)

    initial_x = float(input("Enter initial x-coordinate: "))
    initial_y = float(input("Enter initial y-coordinate: "))
    goal_x = float(input("Enter goal x-coordinate: "))
    goal_y = float(input("Enter goal y-coordinate: "))

    goal_to_goal = GoalToGoal(initial_x, initial_y, goal_x, goal_y)

    # Move forward to the goal
    goal_to_goal.travel_zigzag()

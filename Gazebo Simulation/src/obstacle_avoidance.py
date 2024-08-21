import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

def scan_callback(msg):
    global obstacle_detected, closest_obstacle_angle, closest_obstacle_distance

    # Check if any distance measurement is less than the threshold
    obstacle_distances = msg.ranges
    obstacle_detected = any(dist < 1.5 for dist in obstacle_distances)
    print(obstacle_detected)
    # Find the angle and distance to the closest obstacle
    if obstacle_detected:
        closest_obstacle_index = obstacle_distances.index(min(obstacle_distances))
        closest_obstacle_angle = msg.angle_min + closest_obstacle_index * msg.angle_increment
        closest_obstacle_distance = obstacle_distances[closest_obstacle_index]
    else:
        closest_obstacle_angle = 0.0
        closest_obstacle_distance = 0.0

def avoid_obstacles():
    rospy.init_node('obstacle_avoidance', anonymous=True)
    pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.loginfo("Obstacle Avoidance for TurtleBot3")
    rospy.loginfo("Press 'Ctrl + C' to exit.")

    twist = Twist()

    try:
        while not rospy.is_shutdown():
            if obstacle_detected:
                # Obstacle detected, check if obstacles are on both sides
                left_obstacle = any(dist < 1.5 for dist in obstacle_distances[:len(obstacle_distances)//2])
                print(left_obstacle)
                right_obstacle = any(dist < 1.5 for dist in obstacle_distances[len(obstacle_distances)//2:])
                print(right_obstacle)
                
                if left_obstacle and right_obstacle:
                    # Obstacles on both sides, take reverse while turning
                    twist.linear.x = -0.3  # Reverse
                    twist.angular.z = 0.5  # Turn while reversing
                else:
                    # Obstacle on one side, turn away from the obstacle
                    angle_diff = closest_obstacle_angle
                    twist.linear.x = 0.2  # Move forward
                    twist.angular.z = 0.5 if angle_diff < 0 else -0.5  # Turn away from the obstacle

            else:
                # No obstacle, move forward
                twist.linear.x = 0.2
                twist.angular.z = 0.0

            pub.publish(twist)
            rospy.sleep(0.1)  # Adjust the sleep duration if needed

    except KeyboardInterrupt:
        rospy.loginfo("Shutdown requested. Stopping obstacle avoidance.")
    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.loginfo("Obstacle avoidance stopped.")

if __name__ == '__main__':
    obstacle_detected = False
    closest_obstacle_angle = 0.0
    closest_obstacle_distance = 0.0
    obstacle_distances = []  # Add this line to define the obstacle_distances list
    try:
        avoid_obstacles()
    except rospy.ROSInterruptException:
        pass


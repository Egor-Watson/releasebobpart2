#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan


""""
    Reads lidar from turtle bot.
    
    msg contains an array called ranges with 360 values aka a lidar reading for each degree around the turtle bot.
    Currently we just use the lidar reading from right in front of the turtle bot (0 degrees). 

    TODO: record more lidar reads for more careful avoidance. 
"""
sensor = 0.0
sensor_front_average = 0.0
def lidar_reader(msg):

    # Define sensor as global variable so we can use its value for obstacle avoidance later
    global sensor
    global sensor_front_average

    sensor = msg.ranges[0]
    sensor_front_average = calculateFrontAverage(msg)

    print sensor_front_average

    # print sensor

"""
    Returns the average lidar reading from 315 degrees to 45 degrees. i.e +45 and -45 degrees from the turtlebot's
    front.
"""
def calculateFrontAverage(msg):
    # Probably a better way to do this...
    # msg.ranges is also  tuple, so must convert to list first
    total = sum(list(msg.ranges[:45])) + sum(list(msg.ranges[315:360]))

    return total / 90.0

""""
    Reads current position of the turtle bot.
    Global variables x, y store the current position of the turtle bot.
    Can be used for better avoidance e.g. remember where obstacle was and go around it
"""
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    #theta = msg.pose.pose.angle.z

"""
    A better obstacle avoidance method
"""
def moveDude():

    # If something in front, move a bit back
    while sensor_front_average < 0.8:
        speed.linear.x = -0.2
        speed.linear.y = -0.2

        pub.publish(speed)
        r.sleep()

    speed.linear.x = 0.0
    speed.linear.y = 0.0

    pub.publish(speed)

    # Once enough distance is made between wall and robot, turn around looking for more empty space
    while sensor_front_average < 1.5:
        speed.angular.z = 0.9

        pub.publish(speed)
        r.sleep()

    # Once nothing in front, can move forward safely
    speed.angular.z = 0.0
    speed.linear.x = 0.2
    speed.linear.y = 0.2
    pub.publish(speed)

    # end moveDude()



# Print indicator of start
print "working..."

# Give the node a name (this name is used in the launch file)
rospy.init_node('avoid')

# Subscribe to lidar feed
sub = rospy.Subscriber('/scan', LaserScan, lidar_reader)

# Subscribe to position feed
subPos = rospy.Subscriber("/odom", Odometry, newOdom)

# To communicate speed and rotation we use a Twist object
speed = Twist()

# Publish velocity (used to tell turtle bot which velocity to take)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Publishing rate I think
r = rospy.Rate(4)

# Set a starting speed
speed.linear.x = 0.5
speed.linear.y = 0.5

# Publish starting speed
pub.publish(speed)


# Program loop that checks for obstacles and avoids them
while not rospy.is_shutdown():
    speed.linear.x = 0.2
    speed.linear.y = 0.2

    if sensor_front_average < 0.6:
        moveDude()
    #     speed.linear.x = -0.2
    #     speed.linear.y = -0.2
    #     speed.angular.z = 0.5
    # else:
    #     speed.linear.x = 0.2
    #     speed.linear.y = 0.2
    #     speed.angular.z = 0.0

    # pub.publish(speed)
    r.sleep()

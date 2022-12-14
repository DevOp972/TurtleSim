#!/usr/bin/env python

from turtle import distance
from pid import PID
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtlebot/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtlebot/cmd_vel',
                                                  Twist, queue_size=100)

        # A subscriber to the topic '/turtlebot/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtlebot/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

        self.pid_linear_vel = PID(0.01, 2, 0.001, 1, output_limits=(-100, 100))
        self.pid_ang_vel = PID(0.01, 8, 0.002, 0.1, output_limits=(-60, 60))
        self.d_input = 0
        self.d_input_ang_vel = 0

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.d_input = self.euclidean_distance(data)
        self.d_input_ang_vel = self.steering_angle(data) - self.pose.theta
        # rospy.loginfo(str(self.d_input))
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def linear_vel(self, goal_pose):

        return self.pid_linear_vel.cal(self.d_input, self.euclidean_distance(goal_pose))

    def steering_angle(self, goal_pose):

        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose):

        return self.pid_ang_vel.cal(self.d_input_ang_vel, self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        vel_msg = Twist()
        f = open(
            '/home/smarak/catkin_ws/src/turtlesim_cleaner/src/scaledpts1.txt', "r")

        while not rospy.is_shutdown():

            while(True):
                line = f.readline()

                # Get the input from the file
                pt1 = [float(i) for i in line.split()]
                if(len(pt1) == 0):
                    break

                goal_pose.x = pt1[0]
                goal_pose.y = pt1[1]

                print(goal_pose.x, goal_pose.y)

                while self.euclidean_distance(goal_pose) >= distance_tolerance:

                    vel_msg.linear.x = self.linear_vel(goal_pose)
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = self.angular_vel(goal_pose)
                    print(str(vel_msg.angular.z), str(vel_msg.linear.x),
                          str(self.pose.x), str(self.pose.y))
                    # Publishing our vel_msg
                    self.velocity_publisher.publish(vel_msg)

                    # Publish at the desired rate.
                    self.rate.sleep()

                print("stopped")
                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()


if __name__ == '__main__':
    try:

        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass

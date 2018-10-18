#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

from teste.msg import State

from geometry_msgs.msg import Twist

class turtlebotSwitchState(object):

    def __init__(self):
        rospy.init_node('turtlebot_switch_state', anonymous=False)
        rospy.loginfo('Turtle bot switch state started !')
        rospy.Subscriber('teste/State', State, self.changeState)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state = 3

       
        
    def run_behavior(self):
        while not rospy.is_shutdown():
            try:
                #Wait for messages to be published on turtlebot3/state top
                if (self.state == 0):
                    self.move_forward()
                elif (self.state == 1):
                    self.move_backward()
                elif (self.state == 2):
                    self.rotate_left()

                print(self.state)

                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break

    def changeState(self, msg):
        self.state = msg.state


    def rotate_left(self):
        twist_msg = Twist()
        # liner speed
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # angular speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.3

        self.pub_cmd_vel.publish(twist_msg)


    def move_forward(self):
        twist_msg = Twist()
        # linear speed
        twist_msg.linear.x = 0.5
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # angular speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        self.pub_cmd_vel.publish(twist_msg)

    def move_backward(self):  
        twist_msg = Twist()
        # linear speed
        twist_msg.linear.x = -0.5
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # angular speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0


        self.pub_cmd_vel.publish(twist_msg)

def main():
    my_object = turtlebotSwitchState()
    my_object.run_behavior()

if __name__ == '__main__':
    main()
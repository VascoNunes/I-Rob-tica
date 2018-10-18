#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

from teste.msg import State

from geometry_msgs.msg import Twist

class switchState(object):

    def __init__(self):
        rospy.init_node('switch_state', anonymous=False)
        self.state = rospy.Publisher('teste/State', State, queue_size=1)
        

       
        
    def setState(self):
        while not rospy.is_shutdown():
            try:
                #Wait for messages to be published on turtlebot3/state topic
                n = input('Enter a state: ')


                stateMsg = State()
                stateMsg.state = n
                self.state.publish(stateMsg)
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break

def main():
    my_object = switchState()
    my_object.setState()

if __name__ == '__main__':
    main()
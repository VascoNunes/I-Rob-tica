#!/usr/bin/env python

import rospy

import numpy as np
import tf
from sensor_msgs.msg import LaserScan
from teste.msg import State

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from math import radians, sqrt, pow

class square_movement(object):

    #Odometry_msg(odometry)->Pose_with_covariance(pose)->pose(pose)->point(position)
    def __init__(self):
        rospy.init_node('square_movement', anonymous=False)
        self.state = rospy.Publisher('teste/State', State, queue_size=1)
        self.odometry = rospy.Subscriber('odom', Odometry, self.changeState)
        self.pos = Pose()
        self.movement = 'Straight'
        self.turnAngle = 0
        self.lastAngle = 0 
        
    
       
        
    def setState(self):
        print('Cheguei aqui!!!')
        #stateMsg = State()
        #stateMsg.state = 0
        #self.state.publish(stateMsg)
        print('Cheguei aqui')
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break

    def changeState(self, odometry):
        #0 for moving forward, 1 for moving backwards, 2 for rotating left
        robot_pose = odometry.pose.pose
        #quaternion = np.array([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        #angles = tf.transformations.euler_from_quaternion(quaternion)
        if (self.movement == 'Straight'):
            distance = sqrt(pow((robot_pose.position.x - self.pos.position.x), 2) + pow((robot_pose.position.y - self.pos.position.y), 2))
            if (distance >= 0.5):
                print('Starting to rotate')
                #First we need to stop the robot
                stop = State()
                stop.state = 3
                self.state.publish(stop)
                self.pos.position.x = robot_pose.position.x 
                self.pos.position.y = robot_pose.position.y
                self.lastAngle = getRotation(robot_pose)
                self.movement = 'Rotating'
                #Start rotating
                rotateleft = State()
                rotateleft.state = 2
                self.state.publish(rotateleft)
        else:
            #quaternion = np.array([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
            #angles = tf.transformations.euler_from_quaternion(quaternion)

            yaw = getRotation(robot_pose)
            self.turnAngle = yaw - self.lastAngle
            if ((abs(self.turnAngle + radians(2))) >= radians(90)):
                #First we need to stop the robot
                print(yaw)
                print('90 degrees rotation done')
                stop = State()
                stop.state = 3
                self.state.publish(stop)
                self.movement = 'Straight'
                self.turnAngle = 0
                #Start rotating
                moveForward = State()
                moveForward.state = 0
                self.state.publish(moveForward)





def main():
    my_object = square_movement()
    my_object.setState()


def getRotation(msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw




if __name__ == '__main__':
    main()
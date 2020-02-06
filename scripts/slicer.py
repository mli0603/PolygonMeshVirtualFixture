#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Vector3, Transform
from ros_igtl_bridge.msg import igtlpointcloud, igtltransform, igtlpoint

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist

import datetime

# keyboard
import sys
import select
import tty
import termios

m_to_mm=1000.0

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def rosPoseToArray(pose):
    a = np.eye(4)
    # translation
    a[0, 3] = pose.position.x
    a[1, 3] = pose.position.y
    a[2, 3] = pose.position.z
    # orientation
    quat = np.array([pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w]).astype(np.float64)
    a[:3, :3] = R.from_quat(quat).as_dcm()

    return a

def printMenu():
    print("--------------------")
    print("Press s to start recording")
    print("Press r to re-collect data")

class RegistrationObject():
    def __init__(self):
        self.curr_location = 0
        self.num_fiducials = 4
        self.record_started = False
        self.num_data = 30 #00  # number of data to be collected
        self.pose = np.empty([self.num_fiducials, self.num_data, 4, 4]).astype(np.float64)
        self.counter = 0

        self.registration_point = np.empty([self.num_fiducials,3])
    
    def measuredCPCallBack(self, data):
        msg = igtlpoint()
        msg.name = "Measured"
        msg.pointdata = Point(
            data.pose.position.x*m_to_mm, data.pose.position.y*m_to_mm, data.pose.position.z*m_to_mm)

        self.igtl_point_pub.publish(msg)

        if self.record_started and self.curr_location < self.num_fiducials:
            print(self.counter)

            # convert pose to array
            self.pose[self.curr_location, self.counter,
                      :, :] = rosPoseToArray(data.pose)
            self.counter += 1

            # check for length
            if self.counter >= self.num_data:
                self.counter = 0
                self.record_started = False

                x_avg = self.pose[self.curr_location,:,0,3].mean()
                y_avg = self.pose[self.curr_location,:,1,3].mean()
                z_avg = self.pose[self.curr_location,:,2,3].mean()

                self.registration_point[self.curr_location,:] = np.array([x_avg, y_avg, z_avg])

                self.igtl_point_pub.publish(msg)

                self.curr_location += 1

                printMenu()


    def pub_igtl_point(self):
        msg = igtlpointcloud()
        msg.name = 'Robot Registration'

        msg.pointdata = []
        for i in range(self.registration_point.shape[0]):
            p = Point(self.registration_point[i, 0] * m_to_mm, self.registration_point[i, 1] * m_to_mm, self.registration_point[i, 2] * m_to_mm)
            msg.pointdata.append(p)

        self.igtl_pointcloud_pub.publish(msg)
                

    def servoCPCallBack(self, data):
        msg = igtlpoint()
        msg.name = "Servo"
        msg.pointdata = Point(
            data.pose.position.x*m_to_mm, data.pose.position.y*m_to_mm, data.pose.position.z*m_to_mm)

        self.igtl_point_pub.publish(msg)

    def transformCallback(self, data):
        print('transform')
        print(data)

        # TODO: check name
        transfrom = data.transform
        self.transform_pub.publish(transfrom)

    def registration(self):
        rospy.init_node('ds_registration', anonymous=True)

        # subscribe to robot ee location
        sub_measured = rospy.Subscriber('/PSM2/position_cartesian_current',
                               PoseStamped, self.measuredCPCallBack)
        sub_servo = rospy.Subscriber('PSM2_Proxy/position_cartesian_current',
                               PoseStamped, self.servoCPCallBack)
        transform_sub = rospy.Subscriber(
            '/IGTL_TRANSFORM_IN', igtltransform, self.transformCallback)
        # publish to slicer
        self.igtl_pointcloud_pub = rospy.Publisher(
            '/IGTL_POINTCLOUD_OUT', igtlpointcloud, queue_size=1, latch=True)
        self.igtl_point_pub = rospy.Publisher(
            '/IGTL_POINT_OUT', igtlpoint, queue_size=1, latch=True)

        # publish to robot
        self.transform_pub = rospy.Publisher(
            '/simple_robot/transfrom/ee_to_ds', Transform, queue_size=1, latch=True)

        rate = rospy.Rate(10)  # 10Hz

        printMenu()

        while not rospy.is_shutdown():
            if isData():
                c = sys.stdin.read(1)
                if c == 's' and self.record_started == False:
                    if self.curr_location >= self.num_fiducials:
                        print("--------------------")
                        print("Registration has finished")

                        self.finished = True

                        self.pub_igtl_point()
                    else:
                        self.record_started = True
                        print("--------------------")
                        print("Current location ", self.curr_location)
                        print("Recording started")
                elif c == 'r' and self.record_started == False:
                    self.record_started = True
                    self.curr_location = np.max([0, self.curr_location-1])
                    print("--------------------")
                    print("Re-collect data for location ", self.curr_location)
                    print("Recording started, pivot")
                
            rate.sleep()


if __name__ == "__main__":
    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        obj = RegistrationObject()
        obj.registration()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
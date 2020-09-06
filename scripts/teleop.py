#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import time

# keyboard
import sys
import select
import tty
import termios

import argparse

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def interpolate_setpoints(pt1,pt2,pt3,pt4,sub_interval):
    pts = np.empty([3*sub_interval,3])
    
    pts[:sub_interval,0] = np.linspace(pt1[0],pt2[0],sub_interval)
    pts[:sub_interval,2] = pt1[2]
    
    pts[sub_interval:2*sub_interval,2] = np.linspace(pt2[2],pt3[2],sub_interval)
    pts[sub_interval:2*sub_interval,0] = pt2[0]

    pts[2*sub_interval:3*sub_interval,0] = np.linspace(pt3[0],pt4[0],sub_interval)
    pts[2*sub_interval:3*sub_interval,2] = pt3[2]

    pts[:,1] = pt1[1]

    return pts

def teleop(args):
    pub = rospy.Publisher('/simple_robot/servo_cp', PoseStamped, queue_size=1)
    object_pub = rospy.Publisher('/simple_robot/mesh/file_name', String, queue_size=1, latch=True)

    rospy.init_node('teleop', anonymous=True)
    rate = rospy.Rate(30) # 10Hz
    sub_interval = 200

    # Pose
    t = PoseStamped()
    t.pose.orientation.w = 1.0
    t.pose.position.x = 0.210
    t.pose.position.y = 0.035
    t.pose.position.z = 0.105
    t.header.frame_id = "map"

    if not args.a:
        print("move along x using 4 and 6")
        print("move along y using 8 and 2")
        print("move along z using 7 and 9")
        # increment
        increment = 0.005 # m

        while not rospy.is_shutdown():
            if isData():
                c = sys.stdin.read(1)
                if c == '\x1b':         # x1b is ESC
                    break
                elif c == '4':
                    t.pose.position.x -= increment
                elif c == '6':
                    t.pose.position.x += increment
                elif c == '8':
                    t.pose.position.y += increment
                elif c == '2':
                    t.pose.position.y -= increment
                elif c == '7':
                    t.pose.position.z -= increment
                elif c == '9':
                    t.pose.position.z += increment

            t.header.stamp = rospy.Time.now()
            pub.publish(t)

            rate.sleep()

    else:
        if args.object == 'skull':
            pt1 = np.array([-10,45,35]) / 1000.0 # mm
            pt2 = np.array([ 50,45,35]) / 1000.0 # mm
            pt3 = np.array([ 50,45,85]) / 1000.0 # mm
            pt4 = np.array([-10,45,85]) / 1000.0 # mm

        elif args.object == 'bunny':
            pt1 = np.array([-60,-20,10]) / 1000.0 # mm
            pt2 = np.array([ -5,-20,10]) / 1000.0 # mm
            pt3 = np.array([ -5,-20,50]) / 1000.0 # mm
            pt4 = np.array([-60,-20,50]) / 1000.0 # mm

        elif args.object == 'gear':
            pt1 = np.array([-20, 5, 35]) / 1000.0 # mm
            pt2 = np.array([ 30, 5, 35]) / 1000.0 # mm
            pt3 = np.array([ 30, 5, 65]) / 1000.0 # mm
            pt4 = np.array([-20, 5, 65]) / 1000.0 # mm

        elif args.object == 'concave_cube':
            pt1 = np.array([-35, 20, -80]) / 1000.0 # mm
            pt2 = np.array([ 40, 20, -80]) / 1000.0 # mm
            pt3 = np.array([ 40, 20, -30]) / 1000.0 # mm
            pt4 = np.array([-35, 20, -30]) / 1000.0 # mm

        elif args.object == 'concave_cylinder':
            pt1 = np.array([-65, 40, -20]) / 1000.0 # mm
            pt2 = np.array([-15, 40, -20]) / 1000.0 # mm
            pt3 = np.array([-15, 40,  20]) / 1000.0 # mm
            pt4 = np.array([-65, 40,  20]) / 1000.0 # mm

        elif args.object == 'concave_pyramid':
            pt1 = np.array([-10, -50, 105]) / 1000.0 # mm
            pt2 = np.array([-50, -50, 105]) / 1000.0 # mm
            pt3 = np.array([-50, -50,  45]) / 1000.0 # mm
            pt4 = np.array([-10, -50,  45]) / 1000.0 # mm

        elif args.object == 'concave_sphere':
            pt1 = np.array([-60,  0, -20]) / 1000.0 # mm
            pt2 = np.array([  0,  0, -20]) / 1000.0 # mm
            pt3 = np.array([  0,  0,  10]) / 1000.0 # mm
            pt4 = np.array([-60,  0,  10]) / 1000.0 # mm

        elif args.object == 'convex_cube':
            pt1 = np.array([  50, 20, -80]) / 1000.0 # mm
            pt2 = np.array([ 130, 20, -80]) / 1000.0 # mm
            pt3 = np.array([ 130, 20, -30]) / 1000.0 # mm
            pt4 = np.array([  50, 20, -30]) / 1000.0 # mm
        
        elif args.object == 'convex_cylinder':
            pt1 = np.array([  0, 40, -20]) / 1000.0 # mm
            pt2 = np.array([ 80, 40, -20]) / 1000.0 # mm
            pt3 = np.array([ 80, 40,  20]) / 1000.0 # mm
            pt4 = np.array([  0, 40,  20]) / 1000.0 # mm

        elif args.object == 'convex_pyramid':
            pt1 = np.array([-50, -50,  35]) / 1000.0 # mm
            pt2 = np.array([  0, -50,  35]) / 1000.0 # mm
            pt3 = np.array([  0, -50, 105]) / 1000.0 # mm
            pt4 = np.array([-50, -50, 105]) / 1000.0 # mm

        elif args.object == 'convex_sphere':
            pt1 = np.array([ 25, -15, -20]) / 1000.0 # mm
            pt2 = np.array([ 70, -15, -20]) / 1000.0 # mm
            pt3 = np.array([ 70, -15,  20]) / 1000.0 # mm
            pt4 = np.array([ 25, -15,  20]) / 1000.0 # mm

        # print(filename)
        # object_pub.publish(filename)
        # time.sleep(3)
        set_pts = interpolate_setpoints(pt1,pt2,pt3,pt4,sub_interval)
        
        # send init point
        i = 0
        while i < 300000: 
            t.pose.position.x = pt1[0]
            t.pose.position.y = pt1[1]
            t.pose.position.z = pt1[2]
            
            t.header.stamp = rospy.Time.now()
            pub.publish(t)
            i += 1

        print('motin start')
        
        for pt in set_pts:
            t.pose.position.x = pt[0]
            t.pose.position.y = pt[1]
            t.pose.position.z = pt[2]
            
            t.header.stamp = rospy.Time.now()
            pub.publish(t)

            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--a', action='store_true', help='Autonomous Motion')
    parser.add_argument('--object', type=str, default='skull',help='mesh object to be stored')
    args = parser.parse_args()

    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        teleop(args)
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

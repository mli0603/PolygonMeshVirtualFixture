#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from enum import Enum
from cisst_msgs.msg import prmCartesianImpedanceGains
from cisst_msgs.msg import vctDoubleVec

import ipdb
from rosbag_record import RosbagRecord
import os
import rospkg
import shutil
import math
import subprocess


class StudyControl:
    def __init__(self):
        self.rpack=rospkg.RosPack()
        self.baseFolder=self.rpack.get_path('USAblation')
        self.dataFolder=self.make_user_folder()
        self.mode= -1

        # Define variables
        slaveNameSpace = '/PSM2'
        masterNameSpace = '/MTMR'

        # Rosbag management class
        self.recorder=RosbagRecord('',self.dataFolder,0,slave=slaveNameSpace,master=masterNameSpace)
        
    def stop_recording(self):
        print ("stop recording")
        self.recorder.stop_recording()

    def make_user_folder(self):
        # Get the user name and ID for file recording, create the folder
        failed = True
        while failed:
            try:
                uID=int(input('\nPlease Enter User ID: '))
                failed=False
            except KeyboardInterrupt:
                sys.exit()
            except (ValueError,KeyError):
                print ('\nUser ID must be an integer')
                failed=True
                continue
            dataFolder=os.path.join(self.baseFolder,'data','user' +str(uID))
            if uID == 99:
                self.b_recording = False
            elif os.path.isdir(dataFolder):
                try:
                    yesno=input('\nUser has already been created, do you want to continue (y/n)?')
                except KeyboardInterrupt: 
                    sys.exit()
                failed = (yesno!='y' and yesno!='yes' and yesno!='Y'  and yesno!='Yes') #If user inputs 'y', 'yes', Y, or Yes, failed is false
        if not os.path.exists(dataFolder):
            os.makedirs(dataFolder)

        return dataFolder

    def disable_constraint_motion(self):
        print ("disable_constraint_motion")
        command = 'rostopic pub -1 /PSM2/constraint_motion_status_set std_msgs/Bool "data: false"'
        p = subprocess.Popen(command, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        p.wait(4.0)

    def enable_constraint_motion(self):
        print ("enable_constraint_motion")
        command = 'rostopic pub -1 /PSM2/constraint_motion_status_set std_msgs/Bool "data: true"'
        p = subprocess.Popen(command, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        p.wait(4.0)

    def disable_gravity_compensation(self):
        print ("disable_gravity_compensation")
        command = 'rostopic pub -1 /MTMR/set_gravity_compensation std_msgs/Bool "data: false"'
        p = subprocess.Popen(command, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        p.wait(4.0)

    def enable_gravity_compensation(self):
        print ("enable_gravity_compensation")
        command = 'rostopic pub -1 /MTMR/set_gravity_compensation std_msgs/Bool "data: true"'
        p = subprocess.Popen(command, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        p.wait(4.0)

    def disable_mesh_constraint(self):
        print ("disable_mesh_constraint")
        command = 'rostopic pub -1 /PSM2/mesh_constraint_status_set std_msgs/Bool "data: false"'
        p = subprocess.Popen(command, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        p.wait(4.0)

    def enable_mesh_constraint(self):
        print ("enable_mesh_constraint")
        command = 'rostopic pub -1 /PSM2/mesh_constraint_status_set std_msgs/Bool "data: true"'
        p = subprocess.Popen(command, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        p.wait(4.0)

    def enable_simulation(self):
        print ("enable_simulation")
        command = 'rostopic pub -1 /PSM2/set_simulation std_msgs/Bool "data: true"'
        p = subprocess.Popen(command, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        p.wait(4.0)

    def move_robot_to_start(self):
        print ("move_robot_to_start")
        # switch to teleop mode
        command = 'rostopic pub -1 /eye_robot/set_vRCM std_msgs/Int32 "data: 20"'
        subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')
        command = 'rosrun crtk_teleop PositionPublisher.py'
        subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')

    def teleop_control(self):
        self.disable_constraint_motion()
        self.disable_gravity_compensation()
        print ("teleop")
        self.recorder.new_recording('',method='teleop')

    def teleop_control_vf(self):
        self.enable_constraint_motion()
        self.enable_gravity_compensation()
        print ("teleop with vf")
        self.recorder.new_recording('',method='teleop_vf')
        


def execute_command(switcher,opt):
    if opt >=2 and opt <= 6:
        switcher.mode=opt
    if(inputs_needed[opt]==2):
        inp = input('True/False(default : True): ') or "True"
        truthStatement= inp !='F' and inp !='False'
        functional_calls[opt](switcher,truthStatement)
    elif (inputs_needed[opt]==1):
        functional_calls[opt](switcher)    
    else:
        functional_calls[opt]()
    pass


user_options = ['Quit',
                'Print Options',
                'Enable Constraint Motion',
                'Disable Constraint Motion',
                'Enable Mesh Constraint',
                'Disable Mesh Constraint',
                'Enable Simulation',
                'Teleop',
                'VF + Teleop',
                'Stop Recording']   

    
def print_options():
    i = 0;
    print ('')
    for str in user_options:
        print (i, (' : ' + str))
        i = i+1
        pass
    pass



functional_calls ={0 : quit,
                   1 : print_options,
                   2 : StudyControl.enable_constraint_motion,
                   3 : StudyControl.disable_constraint_motion,
                   4 : StudyControl.enable_mesh_constraint,
                   5 : StudyControl.disable_mesh_constraint,
                   6 : StudyControl.enable_simulation,
                   7 : StudyControl.teleop_control,
                   8 : StudyControl.teleop_control_vf,
                   9 : StudyControl.stop_recording}


inputs_needed ={0 : 0,
                1 : 0,
                2 : 1,
                3 : 1,
                4 : 1,
                5 : 1,
                6 : 1,
                7 : 1,
                8 : 1,
                9 : 1,}


def main():
    #Set up data recording
    
    # initialize ROS node
    rospy.init_node('study_control',disable_signals=True)
    
    switcher=StudyControl()

    opt = '111'
    print_options()
    while int(opt) != 0:
        try:
            opt = int(input('\nEnter option : '))
            execute_command(switcher,opt)
        except  KeyboardInterrupt:
            sys.exit()
        except (ValueError,KeyError) as myerror:
            opt = 1
            print ('Please enter a number, 0 to exit')
            continue
        
        pass

if __name__ == '__main__':
    main()

#!/usr/bin/env python

# Rosbag recording script that allows starting and stopping recording from scriptline
# Is mostly just a hack to run the bash script for "rosbag record" and then to kill all nodes starting with "record_"
# Adapted from https://gist.github.com/marco-tranzatto/8be49b81b1ab371dcb5d4e350365398a
# Which in turn was inspired by responses at https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/

import rospy
import subprocess
import os
import signal
import ipdb, pdb
import std_msgs.msg as std_msgs
import sys
import rospkg


class RosbagRecord:

    def __init__(self,fileInput='',folderInput='',start=False, slave='',master=''):
        
        self.b_recording=False
        self.filename=fileInput
        self.foldername=folderInput
        self.master=master
        self.slave=slave
        
        # If the foldername is empty, use the working directory
        if not self.foldername:
            self.set_foldername(os.getcwd())

        # Make sure to close cleanly on shutdown
        rospy.on_shutdown(self.stop_recording)

        # Start recording
        if start:
           self.start_recording()

   # def __exit__(self,exc_type,exc_value,traceback):
        # self.stop_recording()

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read().decode("utf-8") 
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def set_filename(self,fileInput):
        #TODO: set checks on filename?
        self.filename=fileInput

    def set_foldername(self,folderInput):
        #TODO: set checks on foldername/make it if it doesn't exist
        self.foldername=folderInput

    def start_recording(self,method='all'):
        record_command = 'rosbag record -q --split --duration=4m '
        record_command = record_command + '-e "' + self.master + '/(.*)" -e "'+ self.slave + '/(.*)"'
        
        if not os.path.exists(self.foldername):
            os.makedirs(self.foldername)

        if (not self.b_recording):
            print("recording started")
            # Start recording            
            record_command = record_command + ' -o ' + method
            self.p = subprocess.Popen(record_command, stdin=subprocess.PIPE, shell=True, cwd=self.foldername,
                                      executable='/bin/bash', stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

        self.b_recording=True

    def stop_recording(self):
        if self.b_recording:
            print('stop recording')
            rospy.loginfo(rospy.get_name() + ' stop recording.')
            self.terminate_ros_node("/record_")
            self.p.wait()
        self.b_recording=False

    def new_recording(self,fileInput='',folderInput='',method='all'):
        self.stop_recording()
        if fileInput:
            self.set_filename(fileInput)
        if folderInput:
            self.set_foldername(folderInput)
        self.start_recording(method)
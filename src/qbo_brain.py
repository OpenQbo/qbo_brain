#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2011 Thecorpora, S.L.
#
# This program is free software; you can redistribute it and/or 
# modify it under the terms of the GNU General Public License as 
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program; if not, write to the Free Software 
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
# MA 02110-1301, USA.
#
# Authors: Arturo Bajuelos <arturo@openqbo.com>, 
#          Sergio Merino <sergio.merino@openqbo.com>


import roslib; roslib.load_manifest('qbo_brain')
import rospy

from qbo_listen.msg import Listened
from qbo_talk.srv import Text2Speach

from sensor_msgs.msg import Image
from qbo_face_msgs.msg import FacePosAndDist

import smach
import smach_ros
import signal
import subprocess
import time

global robot_model

#ROS Publishers
global client_speak

global face_detected

def run_process(command = ""):

    if command != "":
        return subprocess.Popen(command.split())
    else:
        return -1

def run_all_process(all_commands):
    proc=[]
    for command in all_commands:
	proc.append(subprocess.Popen(command.split()))
    return proc

def kill_all_process(processes):
    for process in processes:
        process.send_signal(signal.SIGINT)


def speak_this(text):
    global client_speak
    client_speak(str(text))

class RobotModel:
    def __init__(self):
        self.last_object_time = rospy.Time.now()
        self.time_threshold = 0.4
	self.random_move = False
	self.follow_face = False

class CommonQboState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["exit"])
        self.state="none"
        self.input_values={"STOP STATE MACHINE":"exit"}
        self.next_state=""
        self.launchers=[]

    def listen_callback(self, data):
        sentence = data.msg
        rospy.loginfo("Listened: |"+sentence+"|")
	global robot_model
	global face_detected
       


	if self.state=="Default" and sentence == "HALT YOU MOVE":
		
		if robot_model.random_move:
			run_process("rosnode kill /qbo_random_move")
			robot_model.random_move = False
		
		rospy.set_param("/qbo_face_following/move_base", False)
		rospy.follow_face = False
		speak_this("Ok. I stopped moving.")
		return
 
        if self.state=="Default" and not face_detected:
		print "IGNORING PREVIOUS SENTENCE"
		return

	
	if self.state=="Default" and sentence == "WHY DON'T YOU MOVE A ROUND" and not robot_model.random_move:
 		run_process("rosrun qbo_random_move qbo_random_move_face_recog.py")
		robot_model.random_move = True
		speak_this("OK. I'm taking a walk")
	

	elif self.state == "Default" and sentence == "CAN YOU FOLLOW ME" and not robot_model.follow_face:
		if robot_model.random_move:
			run_process("rosnode kill /qbo_random_move")
                	robot_model.random_move = False
		
		speak_this("Yes, I can follow you")
		rospy.follow_face = True
		rospy.set_param("/qbo_face_following/move_base", True)
			


        try:
            self.next_state=self.input_values[data.msg]
        except:
            rospy.loginfo("Sentence not found")
            
#Define default state
class default(CommonQboState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mplayer','phone',''])
        self.state="Default"
        self.input_values={"RUN MUSIC PLAYER":"mplayer", "RUN PHONE SERVICES":"phone"}
        self.launchers=["roslaunch qbo_brain default_state.launch"]
        #self.launchers=[]
        
    def execute(self, userdata): 
        rospy.loginfo('Executing: State '+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)

        #Subscribe to topics
        #Listeners
        subscribe=rospy.Subscriber("/listen/en_default", Listened, self.listen_callback)
        
        #Stereo Selector
        rospy.Subscriber("/qbo_stereo_selector/object", Image, stereo_selector_callback)    
        #Face Tracking
        rospy.Subscriber("/qbo_face_tracking/face_pos_and_dist", FacePosAndDist, face_pos_callback)
        
        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")
                
        subscribe.unregister()
        kill_all_process(pids)
        rospy.loginfo("NextState: "+self.next_state)
        return self.next_state

class musicplayer(CommonQboState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','phone', ''])
        self.state="Music Player"
        self.input_values={"STOP MUSIC PLAYER":"stop", "START PHONE SERVICES":"phone","STOP STATE MACHINE":"exit"}
        self.launchers=["roslaunch qbo_music_player hand_gesture_node.launch"]
        
    def execute(self, userdata): 
        rospy.loginfo('Executing state'+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)
        subscribe=rospy.Subscriber("/listen/en_default", Listened, self.listen_callback)
        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")
                
        speak_this("Exiting music player")
        
	subscribe.unregister()
        kill_all_process(pids)
        rospy.loginfo("NextState:"+self.next_state)
        return self.next_state
        
class phone(CommonQboState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','exit',''])
        self.state="Phone services"
        self.input_values={"STOP PHONE SERVICES":"stop", "STOP STATE MACHINE":"exit"}
        #self.launchers=["roslaunch qbo_http_api_login phone_services.launch"]
        self.launchers=["rosrun qbo_http_api_login qbo_http_api_login.py > /home/qboblue/http_log.txt", "rosrun qbo_mjpeg_server mjpeg_server"]

    def execute(self, userdata): 
        rospy.loginfo('Executing state'+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)
        #run_process("roslaunch qbo_audio_control audio_control_sip.launch")
        subscribe=rospy.Subscriber("/listen/en_default", Listened, self.listen_callback)
   

        speak_this("Phone services are active")
    
        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")
                
        speak_this("Phone services are shut down")

	subscribe.unregister()
        kill_all_process(pids)
        rospy.loginfo("NextState:"+self.next_state)
        #run_process("roslaunch qbo_audio_control audio_control_listener.launch")
        return self.next_state

def stereo_selector_callback(data):
    global robot_model
    
    robot_model.last_object_time = rospy.Time.now()
    
    check_face_object_balance()
       
def face_pos_callback(data):
    global robot_model
    global balance_size
    global face_detected

    face_detected = data.face_detected
 
    check_face_object_balance()
    

def check_face_object_balance():
    global robot_model
    
    diff_time = rospy.Time.now() - robot_model.last_object_time
   
    if rospy.has_param("/qbo_stereo_selector/move_head"):
        object_active = rospy.get_param("/qbo_stereo_selector/move_head")
    else:
        object_active = False


    if object_active and diff_time.to_sec()>robot_model.time_threshold:
        #Need to activate face mode
        rospy.set_param("/qbo_stereo_selector/move_head", False)
        rospy.set_param("/qbo_face_following/move_head", True)
        #print "FACE RECOGNITION MODE"
         
    if (not object_active) and diff_time.to_sec()<robot_model.time_threshold:
        #Need to activate object_mode
        rospy.set_param("/qbo_face_following/move_head", False)
        rospy.set_param("/qbo_stereo_selector/move_head", True)
        #print "OBJECT RECOGNITION MODE"

        
def main():
    global client_speak
    global robot_model
    global face_detected 

    face_detected = False
   
    rospy.init_node("qbo_brain")
    rospy.loginfo("Starting Qbo Brain")
    
    client_speak = rospy.ServiceProxy("/qbo_talk/festival_say_no_wait", Text2Speach)
    rospy.loginfo("Waiting for the qbo_talk service to be active")
    rospy.wait_for_service('/qbo_talk/festival_say_no_wait')
    
    #Initialize robot model
    robot_model = RobotModel()
 
    
    
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])

    with sm:
        smach.StateMachine.add('default', default(), transitions={'mplayer':'music_player','phone':'phoneserver', '':'exit'})
        smach.StateMachine.add('music_player', musicplayer(), transitions={'stop':'default', 'phone':'phoneserver', '':'exit'})
        smach.StateMachine.add('phoneserver', phone(), transitions={'stop':'default', '':'exit'})


    sis= smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()

    speak_this("Q b o brain is active")

    # Execute SMACH plan
    rospy.loginfo("State machine launched")
    outcome = sm.execute()
    rospy.loginfo('Finishing state machine')
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()

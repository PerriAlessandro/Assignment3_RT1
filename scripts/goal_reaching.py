#!/usr/bin/env python

import rospy
import time
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *
from std_msgs.msg import Int32, Bool

#class for the characters colors
class colors:
	PINK = '\033[95m'
	BLUE = '\033[94m'
	CYAN = '\033[96m'
	GREEN = '\033[92m'
	YELLOW = '\033[93m' 
	RED = '\033[91m'
	ORANGE = '\033[33m' 
	PURPLE  = '\033[35m'
	ENDC = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'


msg1 = "Goal Reaching Modality!"
msg2 = "(Press '1' from 'user_interface' console to start this modality)"


currentmode=0
done_cb=False #variable which states the accomplishment of the goal
goal_set=False #variable which states if the goal has already been set
isTimeout=False

def callback_active(): #function which is called when the action starts
    rospy.loginfo("\nAction server is processing the goal...")

def callback_done(state, result): #function which is called when the action is over
	global done_cb
	global goal_set
	if state == 3:
		print(colors.GREEN + colors.UNDERLINE + colors.BOLD + "Goal successfully achieved" + colors.ENDC)
		done_cb = True
		return
	if state == 2:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"PREEMPTED"+ colors.ENDC)
		time.sleep(3)
		os.system('cls||clear') #clear the console
		print (colors.BLUE + colors.UNDERLINE + colors.BOLD +msg1+msg2+colors.ENDC)
		return
	if state == 4:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"ABORTED"+ colors.ENDC)
		return
	if state == 5:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"REJECTED"+ colors.ENDC)
		return
	if state == 6:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"PREEMPTING"+ colors.ENDC)
		return
	if state == 7:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"RECALLING"+ colors.ENDC)
		return
	if state == 8:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"RECALLED"+ colors.ENDC)
		return
	if state == 9:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"LOST"+ colors.ENDC)
		return

def callback_feedback(feedback): #function which is called during the execution of the action
	#return
	rospy.loginfo("Feedback:%s" % str(feedback))

def set_action(): #set-up of the action on the client-side

	global client 
	global goal 
	
	client = actionlib.SimpleActionClient('/move_base',MoveBaseAction) #defining the client
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0

def set_goal(goal_x_coord,goal_y_coord): #function to set the goal
	global goal_set
	global goal
	os.system('cls||clear') #clear the console
	print (colors.BLUE + colors.UNDERLINE + colors.BOLD +msg1+colors.ENDC)
	goal.target_pose.pose.position.x = goal_x_coord
	goal.target_pose.pose.position.y = goal_y_coord
	print(colors.GREEN + colors.UNDERLINE + colors.BOLD +"Desired Position: ("+str(goal_x_coord)+", "+str(goal_y_coord)+")"+colors.ENDC
		)
	client.send_goal(goal,callback_done,callback_active,callback_feedback) #sending the goal

def my_clbk_timeout(event): #function to cancel the goal if its time has expired
	global isTimeout
	if currentmode==1:
		print (colors.RED + colors.UNDERLINE + colors.BOLD +"Goal time expired")
		isTimeout=True
		
	
def mode_callback(data):
    global currentmode
    #rospy.loginfo("I heard %d",data.data)
    currentmode=data.data
    


def main():

	global done_cb
	global goal_set
	global isTimeout
	
	rospy.init_node('goal_reaching')
	pubTimeout=rospy.Publisher('timeout',Bool,queue_size=10)
	pubModality=rospy.Publisher('mode',Int32,queue_size=10)
	subModality=rospy.Subscriber('mode', Int32,mode_callback)
	set_action()
	print (colors.BLUE + colors.UNDERLINE + colors.BOLD +msg1+msg2+colors.ENDC)
	
	while(1):

		if currentmode==1: #if the current mode is '1' i.e. the mode for reaching a goal
			
			if  goal_set==False : #if the goal has not been set yet
				os.system('cls||clear') #clear the console
				print(colors.UNDERLINE + colors.BOLD +"Where do you want the robot to go?"+colors.ENDC)
				goal_x_coord = float(input(colors.BOLD +"Insert the 'x' coordinate of the goal: "+colors.ENDC))
				goal_y_coord = float(input(colors.BOLD +"Insert the 'y' coordinate of the goal: "+colors.ENDC))
				set_goal(goal_x_coord,goal_y_coord)	#set the goal
				goal_set = True
				rospy.Timer(rospy.Duration(60),my_clbk_timeout,True)
			if isTimeout:
				#pubTimeout.publish(True)
				pubModality.publish(0)
				isTimeout=False
				


		else:	#current mode!=1
			
			if goal_set and done_cb==False: #if the goal has been set, the target hasn't been reached yet but the mode has been changed
				client.cancel_goal()
				
			if done_cb: #if the mode has been changed and the task is done
				done_cb=False
			goal_set= False
	rate.sleep()
					
			
     

if __name__ == '__main__':
	main()
        
        
        
        
        
        

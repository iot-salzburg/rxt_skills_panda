#! /usr/bin/env python

import rospy
import time

import actionlib # Brings in the SimpleActionClient
import rxt_skills_panda.msg # Brings in the messages used by the panda actions

#--------------------------------------------------------------------------------------
# client request helper function
#--------------------------------------------------------------------------------------
def send_ROSActionRequest_WithGoal(skillName, skillMsgType, skillGoal):

    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient(skillName, skillMsgType) # Creates SimpleActionClient with skillMsgType action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    client.send_goal(skillGoal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action

    return client.get_result() # Prints out the result (WaitForUserInputResult) of executing the action

#--------------------------------------------------------------------------------------
# main function
#--------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        print ('----------------------------------')
        print ('INVOKING RXT_SKILL: GrabObject')
        result = send_ROSActionRequest_WithGoal('GrabObject', rxt_skills_panda.msg.GrabObjectAction, rxt_skills_panda.msg.GrabObjectGoal(objectPosition=b'1'))
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')

        print ('----------------------------------')
        print ('INVOKING RXT_SKILL: PutObject')
        result = send_ROSActionRequest_WithGoal('PutObject', rxt_skills_panda.msg.PutObjectAction, rxt_skills_panda.msg.PutObjectGoal(position=b'2'))
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')

    except rospy.ROSInterruptException:
        print("program interrupted before completion")

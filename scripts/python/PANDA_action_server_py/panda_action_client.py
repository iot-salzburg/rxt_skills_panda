#! /usr/bin/env python

import rospy
import time

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the panda actions, including the
# goal message and the result message of the task modules "WaitForUserInput" and "VoiceOutput"
import rxt_skills_panda.msg


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# client request implementations of PANDA action server functions
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_request_MoveToLocation(msgBytes):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('MoveToLocation', rxt_skills_panda.msg.MoveToLocationAction) # Creates SimpleActionClient with MoveToLocationAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.MoveToLocationGoal(location=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (MoveToLocationResult) of executing the action


def panda_request_GrabObject(msgBytes):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GrabObject', rxt_skills_panda.msg.GrabObjectAction) # Creates SimpleActionClient with GrabObject action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.GrabObjectGoal(objectPosition=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GrabObjectResult) of executing the action


def panda_request_PutObject(msgBytes):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('PutObject', rxt_skills_panda.msg.PutObjectAction) # Creates SimpleActionClient with PutObject action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.PutObjectGoal(objectPosition=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (PutObjectResult) of executing the action


def panda_request_WaitForUserInput(msgBytes):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForUserInput', rxt_skills_panda.msg.WaitForUserInputAction) # Creates SimpleActionClient with WaitForUserInputAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.WaitForUserInputGoal(inputContent=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForUserInputResult) of executing the action
    
    
def panda_request_WaitForExternalEvent(msgBytes):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForExternalEvent', rxt_skills_panda.msg.WaitForExternalEventAction) # Creates SimpleActionClient WaitForExternalEventAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.WaitForExternalEventGoal(inputText=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForExternalEventResult) of executing the action
    
    
def panda_request_GetData(msgBytes):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GetData', rxt_skills_panda.msg.GetDataAction) # Creates SimpleActionClient with GetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.GetDataGoal(inputData=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GetDataResult) of executing the action


def panda_request_SetData(msgBytes):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('SetData', rxt_skills_panda.msg.SetDataAction) # Creates SimpleActionClient with SetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.SetDataGoal(outputData=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (SetDataResult) of executing the action
	
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:

        # request GrabObject
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: GrabObject')
        result = panda_request_GrabObject(b'1')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')

        # request PutObject
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: PutObject')
        result = panda_request_PutObject(b'1')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')

	    # request MoveToLocation
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: MoveToLocation')
        result = panda_request_MoveToLocation(b'pack pose')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')
        
        # request MoveToLocation
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: MoveToLocation')
        result = panda_request_MoveToLocation(b'cups init')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')
        
        # request MoveToLocation
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: MoveToLocation')
        result = panda_request_MoveToLocation(b'cart init')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')
        
        # request MoveToLocation
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: MoveToLocation')
        result = panda_request_MoveToLocation(b'final cart position')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')
        
        # request GetData
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: GetData')
        result = panda_request_GetData(b'void')
        if result:
            print("Result was:", ', '.join([str(n) for n in result.data.decode("utf-8")]))
        print ('----------------------------------')
        
        # request SetData
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: SetData')
        result = panda_request_SetData(b'data: \'SC 1\'')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')

        # request WaitForUserInput
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: WaitForUserInput')
        result = panda_request_WaitForUserInput(b'void')
        if result:
            print("Result was:", ', '.join([str(n) for n in result.returnMessage.decode("utf-8")]))
        print ('----------------------------------')
                
        # request WaitForExternalEvent
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: WaitForExternalEvent')
        result = panda_request_WaitForExternalEvent(b'void')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')


        # shutdown node
        #print ('----------------------------------')
        #print ('All requests done: Now trying to shutdown everything...')
        #print ('----------------------------------')
        #rospy.signal_shutdown("Finished with success!")
        #rospy.spin()
             
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

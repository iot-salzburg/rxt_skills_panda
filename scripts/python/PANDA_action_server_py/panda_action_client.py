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
def panda_request_MoveToLocation(defGoalLocation):
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('MoveToLocation', rxt_skills_panda.msg.MoveToLocationAction) # Creates SimpleActionClient with MoveToLocationAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.MoveToLocationGoal(location=defGoalLocation) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (MoveToLocationResult) of executing the action


def panda_request_GrabObject():
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GrabObject', rxt_skills_panda.msg.GrabObjectAction) # Creates SimpleActionClient with GrabObject action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.GrabObjectGoal(objectPosition=b'1') # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GrabObjectResult) of executing the action


def panda_request_PutObject():
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('PutObject', rxt_skills_panda.msg.PutObjectAction) # Creates SimpleActionClient with PutObject action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.PutObjectGoal(objectPosition=b'1') # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (PutObjectResult) of executing the action


def panda_request_WaitForUserInput():
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForUserInput', rxt_skills_panda.msg.WaitForUserInputAction) # Creates SimpleActionClient with WaitForUserInputAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.WaitForUserInputGoal(inputContent=b'void') # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForUserInputResult) of executing the action
    
    
def panda_request_WaitForExternalEvent():
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForExternalEvent', rxt_skills_panda.msg.WaitForExternalEventAction) # Creates SimpleActionClient WaitForExternalEventAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.WaitForExternalEventGoal(inputText=b'void') # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForExternalEventResult) of executing the action
    
    
def panda_request_GetData():
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GetData', rxt_skills_panda.msg.GetDataAction) # Creates SimpleActionClient with GetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.GetDataGoal(inputData=b'void') # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GetDataResult) of executing the action


def panda_request_SetData():
    
    rospy.init_node('panda_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('SetData', rxt_skills_panda.msg.SetDataAction) # Creates SimpleActionClient with SetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_panda.msg.SetDataGoal(outputData=b'data: \'SC 1\'') # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (SetDataResult) of executing the action
	
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:

        # request GrabObject
        result = panda_request_GrabObject()
        if result:
            print ('----------------------------------')
            print("Action was: GrabObject")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')

        # request PutObject
        result = panda_request_PutObject()
        if result:
            print ('----------------------------------')
            print("Action was: PutObject")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')

	    # request MoveToLocation
        result = panda_request_MoveToLocation(b'pack pose')
        if result:
            print ('----------------------------------')
            print("Action was: MoveToLocation")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')
        
        # request MoveToLocation
        result = panda_request_MoveToLocation(b'cups init')
        if result:
            print ('----------------------------------')
            print("Action was: MoveToLocation")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')
        
        # request MoveToLocation
        result = panda_request_MoveToLocation(b'cart init')
        if result:
            print ('----------------------------------')
            print("Action was: MoveToLocation")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')
        
        # request MoveToLocation
        result = panda_request_MoveToLocation(b'final cart position')
        if result:
            print ('----------------------------------')
            print("Action was: MoveToLocation")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')
        
        # request GetData
        result = panda_request_GetData()
        if result:
            print ('----------------------------------')
            print("Action was: GetData")
            print("Result was:", ', '.join([str(n) for n in result.data.decode("utf-8")]))
            print ('----------------------------------')
        
        # request SetData
        result = panda_request_SetData()
        if result:
            print ('----------------------------------')
            print("Action was: SetData")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')

        # request WaitForUserInput
        result = panda_request_WaitForUserInput()
        if result:
            print ('----------------------------------')
            print("Action was: WaitForUserInput")
            print("Result was:", ', '.join([str(n) for n in result.returnMessage.decode("utf-8")]))
            print ('----------------------------------')
                
        # request WaitForExternalEvent
        result = panda_request_WaitForExternalEvent()
        if result:
            print ('----------------------------------')
            print("Action was: WaitForExternalEvent")
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

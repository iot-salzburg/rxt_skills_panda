#!/usr/bin/env python
import os, sys, time
import rospy
import actionlib
import rxt_skills_panda.msg


# TODO import required PANDA libs here


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: speak
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_speak(sentence):
    
    print ('TODO: NOT YET IMPLEMENTED!')
    return True
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: listen
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_listen():
    
    print ('TODO: NOT YET IMPLEMENTED!')
    ret=b'TODO'
    return ret

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: move head
#------------------------------------------------------------------------------------------------------------------------------------------------------------ 
def panda_movehead(position):
 
    print ('TODO: NOT YET IMPLEMENTED!')
    return True
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: speak
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_facedetection(inputFace):
    
    print ('TODO: NOT YET IMPLEMENTED!')
    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: speak
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_facialexpression(inputFace):

    print ('TODO: NOT YET IMPLEMENTED!')
    return True
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: write a setting
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_write_setting(setting, value):
    
    print ('TODO: NOT YET IMPLEMENTED!')
    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: read a setting
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_read_setting(setting):
    
    print ('TODO: NOT YET IMPLEMENTED!')
    ret=b'TODO'
    return ret


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# MoveToLocation
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class MoveToLocation(object):
       
    _feedback = rxt_skills_panda.msg.MoveToLocationFeedback() #create feedback message
    _result = rxt_skills_panda.msg.MoveToLocationResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.MoveToLocationAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating MoveToLocation sequence with location %s with seeds %i, %i' % (self._action_name, goal.location, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = panda_movehead(goal.location)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# GrabObject
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class GrabObject(object):
    
    _feedback = rxt_skills_panda.msg.GrabObjectFeedback() #create feedback message
    _result = rxt_skills_panda.msg.GrabObjectResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.GrabObjectAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating GrabObject sequence with object %s with seeds %i, %i' % (self._action_name, goal.objectPosition, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = panda_facialexpression(goal.objectPosition)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# PutObject
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class PutObject(object):
    
    _feedback = rxt_skills_panda.msg.PutObjectFeedback() # create feedback message
    _result = rxt_skills_panda.msg.PutObjectResult() # create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.PutObjectAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating PutObject with object %s with seeds %i, %i' % (self._action_name, goal.objectPosition, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = panda_speak(goal.objectPosition)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
			
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# WaitForUserInput
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class WaitForUserInput(object):
    
    _feedback = rxt_skills_panda.msg.WaitForUserInputFeedback() #create feedback message
    _result = rxt_skills_panda.msg.WaitForUserInputResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.WaitForUserInputAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating WaitForUserInput with input content %s with seeds %i, %i' % (self._action_name, goal.inputContent, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = True
        returnMsg = panda_listen()
          
        if success:
            self._result.returnMessage = returnMsg
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# WaitForExternalEvent
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class WaitForExternalEvent(object):
    
    _feedback = rxt_skills_panda.msg.WaitForExternalEventFeedback() #create feedback message
    _result = rxt_skills_panda.msg.WaitForExternalEventResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.WaitForExternalEventAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating WaitForExternalEvent sequence with inputText %s with seeds %i, %i' % (self._action_name, goal.inputText, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = panda_facedetection(goal.inputText)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# GetData
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class GetData(object):
    
    _feedback = rxt_skills_panda.msg.GetDataFeedback() #create feedback message
    _result = rxt_skills_panda.msg.GetDataResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.GetDataAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating GetData sequence with inputData %s with seeds %i, %i' % (self._action_name, goal.inputData, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = True
        robotName = panda_read_setting(goal.inputData);
          
        if success:
            self._result.data = robotName
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# SetData
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class SetData(object):
    
    _feedback = rxt_skills_panda.msg.SetDataFeedback() #create feedback message
    _result = rxt_skills_panda.msg.SetDataResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.SetDataAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating SetData sequence with outputData %s with seeds %i, %i' % (self._action_name, goal.outputData, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = panda_write_setting('robotName', goal.outputData)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------	 
if __name__ == '__main__':
    
    rospy.init_node('panda')
    server1 = MoveToLocation('MoveToLocation')
    server2 = GrabObject('GrabObject')
    server3 = PutObject('PutObject')
    server4 = WaitForUserInput('WaitForUserInput') 
    server5 = WaitForExternalEvent('WaitForExternalEvent')
    server6 = GetData('GetData')
    server7 = SetData('SetData')
    rospy.spin()
	
	
	

#! /usr/bin/env python3
import os, time
from sre_constants import SUCCESS
import rospy
import actionlib
import rxt_skills_panda.msg

# for publisher subscriber
import std_msgs
from franka_msgs.msg import FrankaState

# self registration
from self_registration import *

# Global variable
sent_message = None
received_message = None

    

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# class containing callback for listening to ros_opcua_response topic
#------------------------------------------------------------------------------------------------------------------------------------------------------------ 
class OPCUA_Response_Listener(object):

    def __init__(self):
        self.flag = True
        rospy.Subscriber('/ros_opcua_response', std_msgs.msg.String, self.listener_callback)

    def listener_callback(self, data):

        message = str(data.data)
        rospy.loginfo(rospy.get_caller_id() + "On Topic opcua_response: I heard message: " + message)

        if (message == 'Stopped'):
            rospy.loginfo("Listener for topic ros_opcua_response will shutdown now")
            self.flag = False
            self.return_value = data.data

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# class containing callback for listening to franka states in order to get robot mode
#------------------------------------------------------------------------------------------------------------------------------------------------------------ 
class FrankaState_RobotMode_Listener(object):

    def __init__(self):
        self.prev = 0
        self.flag = True
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.listener_callback)

    def listener_callback(self, payload):
        
        # end loop if in inputMode we went from white to blue (user finished teaching something)
        if (payload.robot_mode == 1 and self.prev == 5):
            rospy.loginfo("Listener for topic ros_opcua_response will shutdown now")
            self.flag = False
            self.return_value = payload.robot_mode
        
        self.prev = payload.robot_mode
        #rospy.loginfo(rospy.get_caller_id() + "On Topic /franka_states: I heard robot mode: " + str(payload.robot_mode) + 'while previous mode was: ' + str(self.prev))


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: move to location
#------------------------------------------------------------------------------------------------------------------------------------------------------------ 
def panda_move_to_location(location):

    try:   

        rospy.loginfo('Trying to publish to topic: opcua_order')
        if(location.decode("utf-8") == 'pack pose'):
            os.system("rostopic pub -1 -v /ros_opcua_order std_msgs/String \"data: 'ML 1'\"") # 1 ==  pack pose
        elif(location.decode("utf-8") == 'cups init'):
            os.system("rostopic pub -1 -v /ros_opcua_order std_msgs/String \"data: 'ML 2'\"") # 2 ==  cups init
        elif(location.decode("utf-8") == 'cart init'):
            os.system("rostopic pub -1 -v /ros_opcua_order std_msgs/String \"data: 'ML 3'\"") # 3 ==  cart init
        elif(location.decode("utf-8") == 'final cart position 1'):
            os.system("rostopic pub -1 -v /ros_opcua_order std_msgs/String \"data: 'ML 4'\"") # 4 ==  final cart position 1
        elif(location.decode("utf-8") == 'final cart position 2'):
            os.system("rostopic pub -1 -v /ros_opcua_order std_msgs/String \"data: 'ML 5'\"") # 5 ==  final cart position 2
        elif(location.decode("utf-8") == 'final cart position 3'):
            os.system("rostopic pub -1 -v /ros_opcua_order std_msgs/String \"data: 'ML 6'\"") # 6 ==  final cart position 3
        else:
            return False  
        

        rospy.loginfo('Trying to listen from topic: opcua_response')
        list = OPCUA_Response_Listener()

        while list.flag: # sleep to block ActionEnd until we received "Stopped"-message
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass

    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: grab (close gripper)
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_grab(objectPosition):
    
    try:   
        rospy.loginfo('Trying to publish to topic: opcua_order')
        os.system("rostopic pub -1 -v /ros_opcua_order std_msgs/String \"data: 'GO " + objectPosition.decode("utf-8") + "'\"")

        rospy.loginfo('Trying to listen from topic: opcua_response')
        list = OPCUA_Response_Listener()

        while list.flag: # sleep to block ActionEnd until we received "Stopped"-message
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass

    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: put (open gripper)
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_put(objectPosition):
    
    try:   
        rospy.loginfo('Trying to publish to topic: opcua_order')
        os.system("rosrun rxt_panda_tracking track_drop_cup.py "+str(objectPosition.decode("utf-8")))

        rospy.loginfo('Trying to listen from topic: opcua_response')
        list = OPCUA_Response_Listener()

        while list.flag: # sleep to block ActionEnd until we received "Stopped"-message
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass

    return True
   
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: listen for input
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_listen_for_Input():
    
    rospy.loginfo('Trying to listen from topic: franka_states')
    list = FrankaState_RobotMode_Listener()

    while list.flag: # sleep to block ActionEnd until we received message
        rospy.sleep(1)

    ret=b'OK'
    return ret
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: wait external event
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_waitExternalEvent(input):

    rospy.loginfo('Trying to listen from topic: franka_states')
    list = FrankaState_RobotMode_Listener()

    while list.flag: # sleep to block ActionEnd until we received message
        rospy.sleep(1)
    
    return True
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: write a setting
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_write_setting(value):

    # Create subscribers for appropriate topics, custom message and name of callback function. We do not yet get all topics and subscribe to them.
    #rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, publishFrankaState)
    #rospy.Subscriber('/joint_states', JointState, publishJointState)
    #rospy.Subscriber('/panda_pc_heartbeat', NodeExampleData, publish_message)
    #rospy.Subscriber('/exampleWithHeader', NodeExampleDataWithHeader, publish_message)
    #rospy.Subscriber('/exampleWithHeader_throttle', NodeExampleDataWithHeader, publish_message)
    
    rospy.loginfo('Trying to publish to topic: ros_opcua_order')
    pub = rospy.Publisher('/ros_opcua_order', std_msgs.msg.String, queue_size=10)      
    pub.publish(std_msgs.msg.String(value.decode("utf-8")))
    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: read a setting
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def panda_read_setting(value):

    #topics = rospy.get_published_topics()
    #message = str(json.dumps(topics, indent=4))
    #rospy.loginfo("Topics: %s", message)

    rospy.loginfo('Trying to read from topic: ros_opcua_response')
    data = str(rospy.wait_for_message('/ros_opcua_response', std_msgs.msg.String))
    return bytes(data, 'utf-8')



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
        success = panda_move_to_location(goal.location)
          
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
        success = panda_grab(goal.objectPosition)
          
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
        
        rospy.loginfo('%s: Executing, creating PutObject with object %s with seeds %i, %i' % (self._action_name, goal.position, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = panda_put(goal.position)
          
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
        returnMsg = panda_listen_for_Input()
          
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
        success = panda_waitExternalEvent(goal.inputText)
          
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
        print("deteeeeeeect", goal.outputData.decode('utf-8'))

        if goal.outputData == "alexa":
            var = None
            while True:
                var = input("Are you done using alexa ? (y/n): ")
                if var == "y" or var == "Y":
                    self._result.isOK = success
                    rospy.loginfo('%s: Succeeded' % self._action_name)
                    self._as.set_succeeded(self._result)
                    break

        else:        
            # append the seeds to give user feedback
            self._feedback.sequence = []
            self._feedback.sequence.append(0)
            self._feedback.sequence.append(1)
            
            rospy.loginfo('%s: Executing, creating SetData sequence with outputData %s with seeds %i, %i' % (self._action_name, goal.outputData, self._feedback.sequence[0], self._feedback.sequence[1]))
            
            # start executing the action
            #success = fibonacci_example(self, success)
            success = panda_write_setting(goal.outputData)
            
            if success:
                self._result.isOK = success
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# SendMessage
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class SendMessage(object):
    
    _feedback = rxt_skills_panda.msg.SendMessageFeedback() #create feedback message
    _result = rxt_skills_panda.msg.SendMessageResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.SendMessageAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        global sent_message

        # start executing the action
        sent_message = str(goal.messageContent,'utf-8')
        print("the sent message is ",sent_message)
        rospy.sleep(0.2)

         # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating SendMessage sequence with outputData %s with seeds %i, %i' % (self._action_name, goal.messageContent, self._feedback.sequence[0], self._feedback.sequence[1]))

        self._result.isOK = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# OnMessageReceive
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class OnMessageReceive(object):
    
    _feedback = rxt_skills_panda.msg.OnMessageReceiveFeedback() #create feedback message
    _result = rxt_skills_panda.msg.OnMessageReceiveResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_panda.msg.OnMessageReceiveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):

        # start executing the action
        global received_message, sent_message
        received_message = str(goal.messageContent,'utf-8')
        print(sent_message, received_message,"kkkkkkkkkkkkkkkk")
        while sent_message != received_message:
            rospy.sleep(0.1)
            if sent_message == received_message:
                print("exited")
                rospy.sleep(0.2)

                # append the seeds to give user feedback
                self._feedback.sequence = []
                self._feedback.sequence.append(0)
                self._feedback.sequence.append(1)
                
                rospy.loginfo('%s: Executing, creating OnMessageReceive sequence with outputData %s with seeds %i, %i' % (self._action_name, goal.messageContent, self._feedback.sequence[0], self._feedback.sequence[1]))
                
                self._result.isOK = True
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)

                break
            if rospy.is_shutdown() == True:
                break
        

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------	 
if __name__ == '__main__':

    # self registration
    registration_file = loadRegistrationFile()
    uploadAAS(registration_file)
    
    # init server
    rospy.init_node('panda')
    server1 = MoveToLocation('MoveToLocation')
    server2 = GrabObject('GrabObject')
    server3 = PutObject('PutObject')
    server4 = WaitForUserInput('WaitForUserInput') 
    server5 = WaitForExternalEvent('WaitForExternalEvent')
    server6 = GetData('GetData')
    server7 = SetData('SetData')
    server8 = SendMessage('SendMessage')
    server9 = OnMessageReceive('OnMessageReceive')
    rospy.spin()
	
	
	

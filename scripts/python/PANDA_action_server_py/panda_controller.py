#!/usr/bin/env python
import os, sys, time, json

from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# publishFrankaState (see: https://github.com/iot-salzburg/ros_adapter/blob/master/src/ros_kafka_adapter.py)
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def publishFrankaState(payload):

    """builds messages for the working point coordinates (O_T_EE, cols: 13,14,15)
    and the forces in z-axis there (O_F_ext_K, col 3) based on the ros topic '/franka_state_controller/franka_states'"""
    rospy.logdebug(rospy.get_name() + " Type: %s, payload is %s", type(payload), payload)

    # Send if a state change was detected, or every MIN_SENDING_INTERVALL seconds
    actual_franka_state_pos_x = payload.O_T_EE[12]
    actual_franka_state_pos_y = payload.O_T_EE[13]
    actual_franka_state_pos_z = payload.O_T_EE[14]
    if (abs(actual_franka_state_pos_x-ros_status.franka_state_pos_x) > 0.001 or abs(actual_franka_state_pos_y-ros_status.franka_state_pos_y) > 0.001 or 
        abs(actual_franka_state_pos_z-ros_status.franka_state_pos_z) > 0.001 or (time.time() >= ros_status.franka_state_pos_t)):
        # Set next timeout
        if time.time() >= ros_status.franka_state_pos_t:
            ros_status.franka_state_pos_t += MIN_SENDING_INTERVALL
        else:
            ros_status.franka_state_pos_t = time.time() + MIN_SENDING_INTERVALL

        # set the latest states
        ros_status.franka_state_pos_x = actual_franka_state_pos_x
        ros_status.franka_state_pos_y = actual_franka_state_pos_y
        ros_status.franka_state_pos_z = actual_franka_state_pos_z
        
        # try to get the timestamp from the header
        pTime = get_pTime_from_payload(payload)

        # Build and send the message for the positions
        # print("panda.franka_state.pos_x: {}, \tpanda.franka_state.pos_y: {}, \tpanda.franka_state.pos_z: {}".format(
        #    actual_franka_state_pos_x, actual_franka_state_pos_y, actual_franka_state_pos_z))
        pr_client.produce(quantity="panda.franka_state.pos_x", result=actual_franka_state_pos_x, timestamp=pTime)
        pr_client.produce(quantity="panda.franka_state.pos_y", result=actual_franka_state_pos_y, timestamp=pTime)
        pr_client.produce(quantity="panda.franka_state.pos_z", result=actual_franka_state_pos_z, timestamp=pTime)

    # Send the force if a state change was detected, or every MIN_SENDING_INTERVALL seconds
    actual_franka_state_force = payload.O_F_ext_hat_K[2]
    if abs(actual_franka_state_force-ros_status.franka_state_force) > 1 or (time.time() >= ros_status.franka_state_force_t):
        # Set next timeout
        if time.time() >= ros_status.franka_state_force_t:
            ros_status.franka_state_force_t += MIN_SENDING_INTERVALL
        else:
            ros_status.franka_state_force_t = time.time() + MIN_SENDING_INTERVALL

        # set the latest states
        ros_status.franka_state_force = actual_franka_state_force
        
        # try to get the timestamp from the header
        pTime = get_pTime_from_payload(payload)
    
        # Build and send the message for the force
        # print("panda.franka_state.force_z: {}".format(actual_franka_state_force))
        pr_client.produce(quantity="panda.franka_state.force_z", result=actual_franka_state_force, timestamp=pTime)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# publishJointState (see: https://github.com/iot-salzburg/ros_adapter/blob/master/src/ros_kafka_adapter.py)
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def publishJointState(payload):

    """builds messages for the gripper position based on the ros topic '/joint_states'"""
    rospy.logdebug(rospy.get_name() + " Type: %s, payload is %s", type(payload), payload)

    # Extract both gripper positions (index 7 and 8) from payload, sending the sum of both
    actual_panda_gripper_states = payload.position[7] + payload.position[8]

    # Send the force if a state change was detected, or every MIN_SENDING_INTERVALL seconds
    if abs(actual_panda_gripper_states-ros_status.panda_gripper_states) > 0.001 or (time.time() >= ros_status.panda_gripper_states_t):
        # Set next timeout
        if time.time() >= ros_status.panda_gripper_states_t:
            ros_status.panda_gripper_states_t += MIN_SENDING_INTERVALL
        else:
            ros_status.panda_gripper_states_t = time.time() + MIN_SENDING_INTERVALL

        # set the latest states
        ros_status.panda_gripper_states = actual_panda_gripper_states
         # try to get the timestamp from the header
        try:
            pTime = datetime.utcfromtimestamp(payload.header.stamp.secs + payload.header.stamp.nsecs * 1e-9).replace(tzinfo=pytz.UTC).isoformat()
        except Exception as e:
            rospy.logwarn("cannot get timestamp from payload %s", payload)
            rospy.logdebug(e)
            pTime = datetime.utcnow().replace(tzinfo=pytz.UTC).isoformat()
    
        # Build and send the message for the force
        # print("panda.joint_states.gripper_pos: {}".format(actual_panda_gripper_states))
        pr_client.produce(quantity="panda.joint_states.gripper_pos", result=actual_panda_gripper_states, timestamp=pTime)




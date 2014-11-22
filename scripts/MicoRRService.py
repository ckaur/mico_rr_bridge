#!/usr/bin/env python
import roslib
import rospy

import sys, argparse
import struct
import time
import RobotRaconteur as RR
#import thread
#import threading
import numpy

import actionlib
import jaco_driver
import jaco_msgs.srv
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import subprocess

mico_servicedef="""
#Service to provide simple interface to Kinova Mico Arm
service MicoJoint_Interface

option version 0.5

object Mico

function void home_arm()
function void start()
function void stop()

function void setJointAngle(double[] angle_set)
function void setArmPosition(double[] position, double[] orientation)
function void setFingerPosition(double[] position)
end object

"""
class Mico_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('mico_jointstates')
	#print "Enabling Mico Arm"
	#subprocess.call(['roslaunch', 'jaco_driver', 'mico_arm.launch']);

    def home_arm(self):
	print "Homing Arm"
	rospy.wait_for_service('mico_arm_driver/in/home_arm')
	home_arm = rospy.ServiceProxy('mico_arm_driver/in/home_arm', jaco_msgs.srv.HomeArm)
	try:
	    home_arm()
	except rospy.ServiceException as exc:
	    print("Service did not process request: " + str(exc)) 
	
    def start(self):
	print "Starting Arm"
	rospy.wait_for_service('mico_arm_driver/in/start')
	start = rospy.ServiceProxy('mico_arm_driver/in/start', jaco_msgs.srv.Start)
	try:
	    start()
	except rospy.ServiceException as exc:
	    print("Service did not process request: " + str(exc))
	
    def stop(self):
	print "Stopping Arm"
	rospy.wait_for_service('mico_arm_driver/in/stop')
	stop = rospy.ServiceProxy('mico_arm_driver/in/stop', jaco_msgs.srv.Stop)
	try:
	    stop()
	except rospy.ServiceException as exc:
	    print("Service did not process request: " + str(exc))

    def setJointAngle(self, angle_set):
	self._joint_angles = angle_set
	try:
	    result = self._joint_angle_client()
	    print("Done! Set Joint Angles!")
	except rospy.ROSInterruptException:
	    print("Program Interrupted before completion")

    def setArmPosition(self, position, orientation):
	self._arm_pos = position
	self._arm_orient = orientation
	try:
	    result = self._cartesian_pose_client()
	    print("Done! Set Arm Position!")
	except rospy.ROSInterruptException:
	    print("Program Interrupted before completion")

    def setFingerPosition(self, position):
	self._finger_pos = position
	try:
	    result = self._gripper_client()
	    print("Done! Set Finger Position!")
	except rospy.ROSInterruptException:
	    print("Program Interrupted before completion")
	
    def _joint_angle_client(self):
	# Send a joint angle goal to the action server
    	action_address = '/mico_arm_driver/joint_angles/arm_joint_angles'
        client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.ArmJointAnglesAction)
        client.wait_for_server()

    	goal = jaco_msgs.msg.ArmJointAnglesGoal()

    	goal.angles.joint1 = self._joint_angles[0]
    	goal.angles.joint2 = self._joint_angles[1]
    	goal.angles.joint3 = self._joint_angles[2]
    	goal.angles.joint4 = self._joint_angles[3]
    	goal.angles.joint5 = self._joint_angles[4]
    	goal.angles.joint6 = self._joint_angles[5]

    	print('goal: {}'.format(goal))

    	client.send_goal(goal)
    	if client.wait_for_result(rospy.Duration(20.0)):
            return client.get_result()
    	else:
            print('the joint angle action timed-out')
            client.cancel_all_goals()
            return None

    def _cartesian_pose_client(self):
        # Send a cartesian goal to the action server.
        action_address = '/mico_arm_driver/arm_pose/arm_pose'
        client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = jaco_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=('mico_api_origin'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=self._arm_pos[0], y=self._arm_pos[1], z=self._arm_pos[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=self._arm_orient[0], y=self._arm_orient[1], z=self._arm_orient[2], w=self._arm_orient[3])

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('the cartesian action timed-out')
            return None

    def _gripper_client(self):
        # Send a gripper goal to the action server.
        action_address = '/mico_arm_driver/fingers/finger_positions'
        client = actionlib.SimpleActionClient(action_address,
                                              jaco_msgs.msg.SetFingersPositionAction)
        client.wait_for_server()
    
        goal = jaco_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(self._finger_pos[0])
        goal.fingers.finger2 = float(self._finger_pos[1])
    
        # The MICO arm has only two fingers, but the same action definition is used
        if len(self._finger_pos) < 3:
            goal.fingers.finger3 = 0.0
        else:
            goal.fingers.finger3 = float(self._finger_pos[2])
    
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(5.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('the gripper action timed-out')
            return None

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize Joint Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="MicoJointServer"

    #Initialize object
    mico_obj = Mico_impl()

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
                         RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
                         RR.IPNodeDiscoveryFlags_SITE_LOCAL)

    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()

    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(mico_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("Mico", "MicoJoint_Interface.Mico", mico_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/MicoJointServer/Mico"
    raw_input("press enter to quit...\r\n")
    
    # mico_obj.close()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])

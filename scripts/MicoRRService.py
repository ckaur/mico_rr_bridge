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
import sensor_msgs.msg

import subprocess

mico_servicedef="""
#Service to provide simple interface to Kinova Mico Arm
service MicoJoint_Interface

option version 0.5

object Mico
property double[] position
property double[] orientation
property double[] joints_deg
property double[] joints_rad
property double[] fingers

function void home_arm()
function void start()
function void stop()

function void tool_position()
function void joint_angles()
function void joint_state()
function void finger_position()

function void setJointAngle(double[] angle_set)
function void setArmPosition(double[] position, double[] orientation)
function void setFingerPosition(double[] position)
function void carvel(double[] vlinear, double[] vangular, double r, double nc)
function void jointvel(double[] vjoint, double r, double nc)
end object

"""
class Mico_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('mico_jointstates')
        #print "Enabling Mico Arm"
        #subprocess.call(['roslaunch', 'jaco_driver', 'mico_arm.launch']);
        
        # data initializations
        self._arm_pos = [0]*3
        self._arm_orient = [0]*4
        self._joints = [0]*6
        self._joint_angles = [0]*6
        self._finger_pos = [0]*2

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

    def carvel(self, vlinear, vangular, r, nc):
        pub = rospy.Publisher('/mico_arm_driver/in/cartesian_velocity', geometry_msgs.msg.TwistStamped, queue_size=10)
        rate = rospy.Rate(r) # hz
        count = 0
        while not count == nc:
            carvel_msg = geometry_msgs.msg.TwistStamped();
            carvel_msg.header.frame_id	= "/mico_arm_api"
            now = rospy.get_rostime()
            carvel_msg.header.stamp.secs = now.secs
            carvel_msg.header.stamp.nsecs = now.nsecs
            carvel_msg.twist.linear.x = vlinear[0];
            carvel_msg.twist.linear.y = vlinear[1];
            carvel_msg.twist.linear.z = vlinear[2];
            carvel_msg.twist.angular.x = vangular[0];
            carvel_msg.twist.angular.y = vangular[1];
            carvel_msg.twist.angular.z = vangular[2];
            pub.publish(carvel_msg)
            rate.sleep()
            count = count + 1
            print(count)

    def jointvel(self, vjoint, r, nc):
        print("Joint Velocity")
        pub = rospy.Publisher('/mico_arm_driver/in/joint_velocity', jaco_msgs.msg.JointVelocity, queue_size=10)
        rate = rospy.Rate(r) # hz
        count = 0
        while not count == nc:
            jointvel_msg = jaco_msgs.msg.JointVelocity();
            jointvel_msg.joint1 = vjoint[0];
            jointvel_msg.joint2 = vjoint[1];
            jointvel_msg.joint3 = vjoint[2];
            jointvel_msg.joint4 = vjoint[3];
            jointvel_msg.joint5 = vjoint[4];
            jointvel_msg.joint6 = vjoint[5];
            pub.publish(jointvel_msg)
            rate.sleep()
            count = count + 1
            print(count)

    def _toolpos_callback(self, data):
        self._arm_pos[0] = data.pose.position.x
        self._arm_pos[1] = data.pose.position.y
        self._arm_pos[2] = data.pose.position.z
        self._arm_orient[0] = data.pose.orientation.x
        self._arm_orient[1] = data.pose.orientation.y
        self._arm_orient[2] = data.pose.orientation.z
        self._arm_orient[3] = data.pose.orientation.w
        print("Tool Position: ", self._arm_pos, "Tool Orientation: ", self._arm_orient)
    
    def tool_position(self):
        print("Tool Position")
        toolsub = rospy.Subscriber('/mico_arm_driver/out/tool_position', geometry_msgs.msg.PoseStamped, self._toolpos_callback)
        rospy.wait_for_message('/mico_arm_driver/out/tool_position', geometry_msgs.msg.PoseStamped)
        toolsub.unregister()
            
    def _jointang_callback(self, data):
        self._joints[0] = data.joint1
        self._joints[1] = data.joint2
        self._joints[2] = data.joint3
        self._joints[3] = data.joint4
        self._joints[4] = data.joint5
        self._joints[5] = data.joint6
        print("Joint Deg: ", self._joints)
    
    def joint_angles(self):
        print("Joint Angles")
        jointsub = rospy.Subscriber('/mico_arm_driver/out/joint_angles', jaco_msgs.msg.JointAngles, self._jointang_callback)
        rospy.wait_for_message('/mico_arm_driver/out/joint_angles', jaco_msgs.msg.JointAngles)
        jointsub.unregister()

    def _jointstate_callback(self, data):
        self._joint_angles = data.position
        print("Joint Rad: ", self._joint_angles)
    
    def joint_state(self):
        print("Joint State")
        jointstsub = rospy.Subscriber('/mico_arm_driver/out/joint_state', sensor_msgs.msg.JointState, self._jointstate_callback)
        rospy.wait_for_message('/mico_arm_driver/out/joint_state', sensor_msgs.msg.JointState)
        jointstsub.unregister()

    def _finpos_callback(self, data):
        self._finger_pos[0] = data.finger1
        self._finger_pos[1] = data.finger2
        print("Finger Position: ", self._finger_pos)

    def finger_position(self):
        print("Finger Position")
        finsub = rospy.Subscriber('/mico_arm_driver/out/finger_position', jaco_msgs.msg.FingerPosition, self._finpos_callback)
        rospy.wait_for_message('/mico_arm_driver/out/finger_position', jaco_msgs.msg.FingerPosition)
        finsub.unregister()

    @property
    def position(self):
        return self._arm_pos

    @property
    def orientation(self):
        return self._arm_orient

    @property
    def joints_deg(self):
        return self._joints

    @property
    def joints_rad(self):
        return self._joint_angles

    @property
    def fingers(self):
        return self._finger_pos

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

#!/usr/bin/env python
import roslib
import rospy

import sys, argparse
import struct
import time
import RobotRaconteur as RR
# import thread
import threading
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
# Service to provide simple interface to Kinova Mico Arm
service MicoJoint_Interface

option version 0.5

object Mico
property double[] position
property double[] orientation
property double[] joints_deg
property double[] joints_rad
property double[] joints_vel
property double[] fingers
property double[] force
property double[] torque

function void home_arm()
function void start()
function void stop()
function void start_force_control()
function void stop_force_control()
function void force_control_params(double inertia_lin, double inertia_ang, double damping_lin, double damping_ang, double force_min_lin, double force_min_ang, double force_max_lin, double force_max_ang)

function void modify_rate(double[] command_rate)

function void setJointAngle(double[] angle_set)
function void setArmPosition(double[] position, double[] orientation)
function void setFingerPosition(double[] position)
function void carvel(double[] vlinear, double[] vrotational)
function void jointvel(double[] vjoint)
end object

"""
class Mico_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('mico_jointstates')

        # TODO: Try to enable Mico Arm from this script
        # print "Enabling Mico Arm"
        # subprocess.call(['roslaunch', 'jaco_driver', 'mico_arm.launch']);
        
        # data initializations
        self._arm_pos = [0]*3
        self._arm_orient = [0]*4
        self._joints = [0]*6
        self._joint_angles = [0]*6
        self._joint_velocities = [0]*6
        self._finger_pos = [0]*2
        self._force = [0]*3
        self._torque = [0]*3
        self._jointvel_msg = jaco_msgs.msg.JointVelocity()
        self._carvel_msg = geometry_msgs.msg.TwistStamped();

        # command rate initialization
        self._command_rate = 100

        # initialize subscribers
        self.tool_position()
        self.joint_angles()
        self.joint_state()
        self.finger_position()
        self.tool_wrench()

        # start background threads
        self._running = True
        self._t_command = threading.Thread(target=self.command_worker)
        self._t_command.daemon = True
        self._t_command.start()

    def close(self):
        print "Closing Node"
        self._running = False
        self._t_command.join()

        self._toolsub.unregister()
        self._jointsub.unregister()
        self._jointstsub.unregister()
        self._finsub.unregister()
        self._wrenchsub.unregister()

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

    def start_force_control(self):
        print "Starting Force Control"
        rospy.wait_for_service('mico_arm_driver/in/start_force_control')
        start_force_control = rospy.ServiceProxy('mico_arm_driver/in/start_force_control', jaco_msgs.srv.Start)
        try:
            start_force_control()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def stop_force_control(self):
        print "Stopping Force Control"
        rospy.wait_for_service('mico_arm_driver/in/stop_force_control')
        stop_force_control = rospy.ServiceProxy('mico_arm_driver/in/stop_force_control', jaco_msgs.srv.Stop)
        try:
            stop_force_control()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def force_control_params(self, inertia_lin, inertia_ang, damping_lin, damping_ang, force_min_lin, force_min_ang, force_max_lin, force_max_ang):
        print "Setting Force Control Parameters"
        rospy.wait_for_service('mico_arm_driver/in/set_force_control_params')
        set_force_control_params = rospy.ServiceProxy('mico_arm_driver/in/set_force_control_params', jaco_msgs.srv.SetForceControlParams)

        set_req = jaco_msgs.srv.SetForceControlParamsRequest()
        set_req.inertia_linear.x = inertia_lin
        set_req.inertia_linear.y = inertia_lin
        set_req.inertia_linear.z = inertia_lin
        set_req.inertia_angular.x = inertia_ang
        set_req.inertia_angular.y = inertia_ang
        set_req.inertia_angular.z = inertia_ang
        set_req.damping_linear.x = damping_lin
        set_req.damping_linear.y = damping_lin
        set_req.damping_linear.z = damping_lin
        set_req.damping_angular.x = damping_ang
        set_req.damping_angular.y = damping_ang
        set_req.damping_angular.z = damping_ang
        set_req.force_min_linear.x = force_min_lin
        set_req.force_min_linear.y = force_min_lin
        set_req.force_min_linear.z = force_min_lin
        set_req.force_min_angular.x = force_min_ang
        set_req.force_min_angular.y = force_min_ang
        set_req.force_min_angular.z = force_min_ang
        set_req.force_max_linear.x = force_max_lin
        set_req.force_max_linear.y = force_max_lin
        set_req.force_max_linear.z = force_max_lin
        set_req.force_max_angular.x = force_max_ang
        set_req.force_max_angular.y = force_max_ang
        set_req.force_max_angular.z = force_max_ang

        try:
            set_force_control_params(set_req)
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
            print("the joint angle action timed-out")
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
            print("the cartesian action timed-out")
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
            print("the gripper action timed-out")
            return None

    def carvel(self, vlinear, vrotational):
        now = rospy.get_rostime()
        self._carvel_msg.header.frame_id = "/mico_arm_api"
        self._carvel_msg.header.stamp.secs = now.secs
        self._carvel_msg.header.stamp.nsecs = now.nsecs
        self._carvel_msg.twist.linear.x = vlinear[0];
        self._carvel_msg.twist.linear.y = vlinear[1];
        self._carvel_msg.twist.linear.z = vlinear[2];
        self._carvel_msg.twist.angular.x = vrotational[0];
        self._carvel_msg.twist.angular.y = vrotational[1];
        self._carvel_msg.twist.angular.z = vrotational[2];

    def jointvel(self, vjoint):
        self._jointvel_msg.joint1 = vjoint[0];
        self._jointvel_msg.joint2 = vjoint[1];
        self._jointvel_msg.joint3 = vjoint[2];
        self._jointvel_msg.joint4 = vjoint[3];
        self._jointvel_msg.joint5 = vjoint[4];
        self._jointvel_msg.joint6 = vjoint[5];

    def _toolpos_callback(self, data):
        self._arm_pos[0] = data.pose.position.x
        self._arm_pos[1] = data.pose.position.y
        self._arm_pos[2] = data.pose.position.z
        self._arm_orient[0] = data.pose.orientation.x
        self._arm_orient[1] = data.pose.orientation.y
        self._arm_orient[2] = data.pose.orientation.z
        self._arm_orient[3] = data.pose.orientation.w
    
    def tool_position(self):
        self._toolsub = rospy.Subscriber('/mico_arm_driver/out/tool_position', geometry_msgs.msg.PoseStamped, self._toolpos_callback)
            
    def _jointang_callback(self, data):
        self._joints[0] = data.joint1
        self._joints[1] = data.joint2
        self._joints[2] = data.joint3
        self._joints[3] = data.joint4
        self._joints[4] = data.joint5
        self._joints[5] = data.joint6
   
    # raw joint angles in degrees 
    def joint_angles(self):
        self._jointsub = rospy.Subscriber('/mico_arm_driver/out/joint_angles', jaco_msgs.msg.JointAngles, self._jointang_callback)

    def _jointstate_callback(self, data):
        self._joint_angles = data.position
        self._joint_velocities = data.velocity
    
    # joint angles in radians
    def joint_state(self):
        self._jointstsub = rospy.Subscriber('/mico_arm_driver/out/joint_state', sensor_msgs.msg.JointState, self._jointstate_callback)

    def _finpos_callback(self, data):
        self._finger_pos[0] = data.finger1
        self._finger_pos[1] = data.finger2

    def finger_position(self):
        self._finsub = rospy.Subscriber('/mico_arm_driver/out/finger_position', jaco_msgs.msg.FingerPosition, self._finpos_callback)

    def _toolwrench_callback(self, data):
        self._force[0] = data.wrench.force.x
        self._force[1] = data.wrench.force.y
        self._force[2] = data.wrench.force.z
        self._torque[0] = data.wrench.torque.x
        self._torque[1] = data.wrench.torque.y
        self._torque[2] = data.wrench.torque.z

    def tool_wrench(self):
        self._wrenchsub = rospy.Subscriber('/mico_arm_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, self._toolwrench_callback)

    # function to modify the command rate
    def modify_rate(self, command_rate):
        self._command_rate = command_rate

    # worker function to continuously issue commands to baxter
    # try to maintain user defined operation rate
    def command_worker(self):
        carvel_pub = rospy.Publisher('/mico_arm_driver/in/cartesian_velocity', geometry_msgs.msg.TwistStamped, queue_size=1)
        jointvel_pub = rospy.Publisher('/mico_arm_driver/in/joint_velocity', jaco_msgs.msg.JointVelocity, queue_size=1)
        while self._running:
            rate = rospy.Rate(self._command_rate)
            carvel_pub.publish(self._carvel_msg)
            jointvel_pub.publish(self._jointvel_msg)
            rate.sleep()

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
    def joints_vel(self):
        return self._joint_velocities

    @property
    def fingers(self):
        return self._finger_pos

    @property
    def force(self):
        return self._force

    @property
    def torque(self):
        return self._torque

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize Joint Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    # Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    # Set the Node name
    RR.RobotRaconteurNode.s.NodeName="MicoJointServer"

    # Initialize object
    mico_obj = Mico_impl()

    # Create transport, register it, and start the server
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

    # Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(mico_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("Mico", "MicoJoint_Interface.Mico", mico_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/MicoJointServer/Mico"
    raw_input("press enter to quit...\r\n")
    
    mico_obj.close()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])

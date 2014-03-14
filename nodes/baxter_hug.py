#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_kinect_calibration")

import argparse
import sys
import rospy
import baxter_interface

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
global left_arm, right_arm

def hug(height, strength):
    baxter_interface.RobotEnable().enable()
    left_arm = baxter_interface.limb.Limb("left")
    right_arm = baxter_interface.limb.Limb("right")
    hug_prep_right = Pose(position=Point(x=0.9,y=-0.7,z=height), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))    
    hug_prep_left = Pose(position=Point(x=0.9,y=0.7,z=height), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707)) 
  

    start_left = solveIK(hug_prep_left, "left")
    start_right = solveIK(hug_prep_right, "right")
    left_arm.move_to_joint_positions(start_left)
    right_arm.move_to_joint_positions(start_right)
    
    hug_final_right = Pose(position=Point(x=0.4,y=0.5,z=height - 0.1), orientation=Quaternion(x=-0.5,y=0.5,z=0.5,w=0.5))    
    hug_final_left = Pose(position=Point(x=0.4,y=-0.5,z=height+0.1), orientation=Quaternion(x=0.5,y=0.5,z=-0.5,w=0.5)) 
    end_left = solveIK(hug_final_left, "left")
    end_right = solveIK(hug_final_right, "right")
    joints_left = getIntermediateJointPositions(start_left, end_left)
    joints_right = getIntermediateJointPositions(start_right, end_right)
    moveJoints(joints_left, joints_right)
    rospy.sleep(3)

    joints_left = getIntermediateJointPositions(end_left, start_left)
    joints_right = getIntermediateJointPositions(end_right, start_right)
    moveJoints(joints_left, joints_right)
    rospy.sleep(3)
    left_arm.move_to_neutral()
    right_arm.move_to_neutral()    

def moveJoints(left, right):
    left_arm = baxter_interface.limb.Limb("left")
    right_arm = baxter_interface.limb.Limb("right")
    count = min(len(left), len(right))
    for i in range(count):
        moveJointsToPosition(left_arm, left[i])
        moveJointsToPosition(right_arm, right[i])
        rospy.sleep(0.1)

def moveJointsToPosition(limb, setPoint):
    #left_arm.set_joint_positions(left[i])
    #right_arm.set_joint_positions(right[i])
    springs = getSprings(limb)
    damping = getDamping(limb)
    cur_pos = limb.joint_angles()
    cur_vel = limb.joint_velocities()
    cmd = dict()
    for joint in limb.joint_names():
        cmd[joint] = -springs[joint] * (cur_pos[joint] - setPoint[joint])
        cmd[joint] -= damping[joint] * cur_vel[joint]
    limb.set_joint_torques(cmd)

def getSprings(limb):
    springs = dict()
    for joint in limb.joint_names():
        springs[joint] = .0001
    return springs
    
def getDamping(limb):
        damping = dict()
        for joint in limb.joint_names():
            damping[joint] = 1
        return damping

def getIntermediateJointPositions(start, end, numSteps = 100):
    joints = []
    for i in range(numSteps):
        t = float(i) / numSteps;
        jointState = dict()
        for key in start:
           jointState[key] = start[key] + t*(end[key] - start[key])
        joints.append(jointState)
    return joints

def solveIK(pose, limb):
    ns = "/ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    goalPose = PoseStamped(header=hdr, pose=pose)


    ikreq.pose_stamp.append(goalPose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints;
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
  
if __name__ == '__main__':
    rospy.init_node("baxter_hug")
    hug(0.4, 10.0)
    

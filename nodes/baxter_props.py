#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_props")

import argparse
import sys
import rospy
import baxter_interface

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from dynamic_reconfigure.server import Server
from baxter_examples.cfg import JointSpringsExampleConfig

class BaxterProps:
    def __init__(self):
        self.dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    def hug(self, height, strength):
        baxter_interface.RobotEnable().enable()
        left_arm = baxter_interface.limb.Limb("left")
        right_arm = baxter_interface.limb.Limb("right")
        hug_prep_right = Pose(position=Point(x=0.9,y=-0.7,z=height), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))    
        hug_prep_left = Pose(position=Point(x=0.9,y=0.7,z=height), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707)) 
      
        cur_left = left_arm.joint_angles()
        cur_right = right_arm.joint_angles()   
        start_left = self.solveIK(hug_prep_left, "left")
        start_right = self.solveIK(hug_prep_right, "right")
        joints_left = self.getIntermediateJointPositions(cur_left, start_left)
        joints_right = self.getIntermediateJointPositions(cur_right, start_right)
        self.moveTwoArms(left_arm, right_arm, joints_left, joints_right, False)
        
        hug_final_right = Pose(position=Point(x=0.4,y=0.5,z=height - 0.1), orientation=Quaternion(x=-0.5,y=0.5,z=0.5,w=0.5))    
        hug_final_left = Pose(position=Point(x=0.4,y=-0.5,z=height + 0.1), orientation=Quaternion(x=0.5,y=0.5,z=-0.5,w=0.5)) 
        end_left = self.solveIK(hug_final_left, "left")
        end_right = self.solveIK(hug_final_right, "right")
        joints_left = self.getIntermediateJointPositions(start_left, end_left)
        joints_right = self.getIntermediateJointPositions(start_right, end_right)
        self.moveTwoArms(left_arm, right_arm, joints_left, joints_right, True)
        rospy.sleep(3)

        cur_left = left_arm.joint_angles()
        cur_right = right_arm.joint_angles()   
        joints_left = self.getIntermediateJointPositions(cur_left, start_left)
        joints_right = self.getIntermediateJointPositions(cur_right, start_right)
        self.moveTwoArms(left_arm, right_arm, joints_left, joints_right, False)

    def bump(self, point, limb):
        baxter_interface.RobotEnable().enable()
        arm = baxter_interface.limb.Limb(limb)
        prep = Pose(position=Point(x=point.x-0.2,y=point.y,z=point.z), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))
        cur_angles = arm.joint_angles()
        start = self.solveIK(prep, limb)
        joints = self.getIntermediateJointPositions(cur_angles, start)
        self.moveOneArm(arm, joints, False)
        
        final = Pose(position=point, orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))    
        end = self.solveIK(final, limb)
        joints = self.getIntermediateJointPositions(start, end)
        self.moveOneArm(arm, joints, False)
        rospy.sleep(3)

        cur_angles = arm.joint_angles()
        joints = self.getIntermediateJointPositions(cur_angles, start)
        self.moveOneArm(arm, joints, False)

    def five(self, point, limb):
        baxter_interface.RobotEnable().enable()
        arm = baxter_interface.limb.Limb(limb)
        prep = Pose(position=Point(x=point.x-0.2,y=point.y,z=point.z), orientation=Quaternion(x=0.707,y=0,z=0,w=0.707))
        cur_angles = arm.joint_angles()
        start = self.solveIK(prep, limb)
        joints = self.getIntermediateJointPositions(cur_angles, start)
        self.moveOneArm(arm, joints, False)
        
        final = Pose(position=point, orientation=Quaternion(x=0.707,y=0,z=0,w=0.707))    
        end = self.solveIK(final, limb)
        joints = self.getIntermediateJointPositions(start, end)
        self.moveOneArm(arm, joints, False)
        rospy.sleep(3)

        cur_angles = arm.joint_angles()
        joints = self.getIntermediateJointPositions(cur_angles, start)
        self.moveOneArm(arm, joints, False)

    def moveOneArm(self, arm, joints, useTorqueMode):
        count = len(joints)
        control_rate = rospy.Rate(1000)

        for i in range(count):
            self.moveJointsToPosition(arm, joints[i], useTorqueMode)
            control_rate.sleep()

        arm.exit_control_mode()

    def moveTwoArms(self, left_arm, right_arm, left_angles, right_angles, useTorqueMode):
        count = min(len(left_angles), len(right_angles))
        control_rate = rospy.Rate(1000)

        for i in range(count):
            self.moveJointsToPosition(left_arm, left_angles[i], useTorqueMode)
            self.moveJointsToPosition(right_arm, right_angles[i], useTorqueMode)
            control_rate.sleep()

        left_arm.exit_control_mode()
        right_arm.exit_control_mode()

    def moveJointsToPosition(self, limb, setPoint, useTorqueMode):
        if useTorqueMode:
            [springs, damping] = self.getSprings(limb)
            cur_pos = limb.joint_angles()
            cur_vel = limb.joint_velocities()
            cmd = dict()
            for joint in limb.joint_names():
                cmd[joint] = springs[joint] * (setPoint[joint]- cur_pos[joint])
                cmd[joint] -= damping[joint] * cur_vel[joint]
            limb.set_joint_torques(cmd)
        else:
            limb.set_joint_positions(setPoint)

    def getSprings(self, limb):
        
        springs = dict()
        damping = dict()
        for joint in limb.joint_names():
            springs[joint] = self.dynamic_cfg_srv.config[joint[-2:] +
                                                        '_spring_stiffness']
            damping[joint] = self.dynamic_cfg_srv.config[joint[-2:] +
                                                        '_damping_coefficient']
        return [springs, damping]


    def getIntermediateJointPositions(self, start, end, numSteps = 10000):
        joints = []
        for i in range(numSteps):
            t = float(i) / numSteps;
            jointState = dict()
            for key in start:
               jointState[key] = start[key] + t*(end[key] - start[key])
            joints.append(jointState)
        return joints


    def solveIK(self, pose, limb):
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
    rospy.init_node("baxter_props")
    props = BaxterProps()
    #props.hug(0.4, 10.0)
    #props.bump(Point(x=0.7, y=0.7, z=0), "left")
    props.five(Point(x=0.5, y=0.7, z=0.5), "left")
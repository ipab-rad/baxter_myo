import sys
from copy import deepcopy
import time
import math

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Twist,
    Quaternion,
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_myo.msg import MyoData

class PoseGenerator(object):

    def __init__(self, limb, mode, init_pose):
        self.name = limb
        self.mode = mode
        self.init_pose = init_pose
        self._limb = baxter_interface.Limb(self.name)
        self.calibrated = False
        self.enabled = False
        self.gripper_enabled = False
        self.data = Twist()
        rospy.Subscriber("myo_data", MyoData, self.myo_data_callback)
        rospy.loginfo("Subscribed to myo_data")
        self.received = False
        # IK service stuff
        self._iksrvp = rospy.ServiceProxy(
            "ExternalTools/" + self.name + "/PositionKinematicsNode/IKService",
            SolvePositionIK)
        self._ik_request = SolvePositionIKRequest()
        self._closed = False

    def myo_data_callback(self, msg):
        self.received = True
        self.gripper_enabled = data.gripper
        self.calibrated = data.calibrated
        self.enabled = data.enabled
        self.data = deepcopy(data.data)

    def gen_next(self):
        if self.mode == "positions":
            return gen_positions(self)

    def find_joint_positions(self, pose,
                             pos_x=0.0, pos_y=0.0, pos_z=0.0,
                             ori_x=0.0, ori_y=0.0, ori_z=0.0):
        '''
        Finds the joint position of the arm given some pose and the
        offsets from it
        '''
        new_pose = Point(x=pose['position'].x + pos_x,
                         y=pose['position'].y + pos_y,
                         z=pose['position'].z + pos_z)
        new_orientation = Point(x=pose['orientation'].x + ori_x,
                                y=pose['orientation'].y + ori_y,
                                z=pose['orientation'].z + ori_z)
        approach_pose = Pose()
        approach_pose.position = new_pose
        approach_pose.orientation = new_orientation
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        self._ik_request.pose_stamp.append(pose_req)
        resp = self._iksrvp(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def gen_positions(self):
        if not self.enabled:
            rospy.logwarn("Myo is not enabled")
            return ("not_enabled", None)
        elif not self.calibrated:
            rospy.logwarn("Myo is not calibrated")
            return ("not_calibrated", None)
        elif self.received:
            pose_diff = self.calc_pos(self.data.linear)
            orient_diff = self.calc_orient(self.data.angular)
            px = pose_diff[0]
            py = pose_diff[1]
            pz = pose_diff[2]
            ox = orient_diff[0]
            oy = orient_diff[1]
            oz = orient_diff[2]
            new_poss = self.find_joint_pose(
                self._limb.endpoint_pose(),
                targ_x=px,
                targ_y=py,
                targ_z=pz,
                targ_ox=ox,
                targ_oy=oy,
                targ_oz=oz)
            self.received = False
            return ("new_poss", new_poss)

    def check_gripper():
        """
        Returns True if gripper has to be closed. Otherwise returns
        False.
        """
        if self.gripper_enabled:
            if not self._closed:
                rospy.loginfo("Closing gripper")
                self._closed = True
                return True
        else:
            if self._closed:
                rospy.loginfo("Opening gripper")
                self._closed = False
                return False

    def calc_pos(self, sensor_pose):
        sensed_pos_diff = self._pos_diff(self.init_pose,
                                         sensor_pose)
        actual_pos_diff = self._pos_diff(self.init_pose,
                                         self._limb.endpoint_pose())
        diff = self._pos_diff(sensed_pos_diff, actual_pos_diff)
        # diff[2] = -diff[2]
        return diff

    def _pos_diff(self, p1, p2):
        try:
            cp = p1['position']
            ip = p2['position']
            pose_diff = []
            pose_diff.append(cp.x - ip.x)
            pose_diff.append(cp.y - ip.y)
            pose_diff.append(cp.z - ip.z)
            return pose_diff
        except:
            return [x - y for (x, y) in zip(p1, p2)]

    def calc_orient(self, sensor_pose):
        sensed_orient_diff = self._orient_diff(self.init_pose,
                                               sensor_pose)
        actual_orient_diff = self._orient_diff(self.init_pose,
                                               self._limb.endpoint_pose())
        diff = self._orient_diff(sensed_orient_diff, actual_orient_diff)
        # diff[2] = -diff[2]
        return diff

    def _orient_diff(self, p1, p2, ):
        try:
            cp = p1['orientation']
            ip = p2['orientation']
            orient_diff = []
            orient_diff.append(cp.x - ip.x)
            orient_diff.append(cp.y - ip.y)
            orient_diff.append(cp.z - ip.z)
            return orient_diff
        except:
            return [x - y for (x, y) in zip(p1, p2)]

    def gen_angles(self):
        """
        Set angles given data from two Myos.
        Returns True if angles can be set, otherwise returns False
        """
        limb_name = self.name_limb
        if self.high_received:
            e0 = math.radians(self.high_data.angular.x)
            s1 = math.radians(self.high_data.angular.y)
            s0 = math.radians(self.high_data.angular.z)
            new_poss[limb_name + '_e0'] = -e0
            new_poss[limb_name + '_s1'] = s1
            new_poss[limb_name + '_s0'] = s0

        if self.low_received:
            w0 = math.radians(self.low_data.angular.x)
            e1 = math.radians(self.low_data.angular.y)
            w1 = math.radians(self.low_data.angular.z)
            new_poss[limb_name + '_w0'] = -w0
            new_poss[limb_name + '_e1'] = e1
            new_poss[limb_name + '_w1'] = w1
        return new_poss

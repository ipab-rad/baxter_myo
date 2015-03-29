import sys
import time
import math
from copy import deepcopy

import rospy
import tf
from std_msgs.msg import Header
from std_msgs.msg import Bool
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Twist,
    Quaternion,
    Vector3
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_myo.msg import MyoData

class PoseGenerator(object):

    def __init__(self, limb, mode, init_pose):
        self.limb_name = limb
        self.mode = mode
        self.init_pose = init_pose
        self._limb = baxter_interface.Limb(self.limb_name)
        self.calibrated = False
        self.enabled = False
        self.gripper_enabled = False
        self.data = Twist()
        # rospy.Subscriber("myo_data", MyoData, self.myo_data_callback)
        # rospy.loginfo("Subscribed to myo_data")
        rospy.Subscriber("position_myo_0", Vector3, self.position_callback)
        rospy.loginfo("Subscribed to myo_data")
        rospy.Subscriber("orientation_myo_0", Vector3, self.orientation_callback)
        rospy.loginfo("Subscribed to myo_data")
        rospy.Subscriber("enabled_myo_0", Bool, self.enabled_callback)
        rospy.loginfo("Subscribed to myo_data")
        rospy.Subscriber("calibrated_myo_0", Bool, self.calibrated_callback)
        rospy.loginfo("Subscribed to myo_data")
        self.received = False
        # IK service stuff
        self._iksrvp = rospy.ServiceProxy(
            "ExternalTools/" + self.limb_name
            + "/PositionKinematicsNode/IKService",
            SolvePositionIK)
        self._ik_request = SolvePositionIKRequest()
        self._closed = False

    def myo_data_callback(self, msg):
        self.received = True
        # self.gripper_enabled = data.gripper
        # self.calibrated = data.calibrated
        # self.enabled = data.enabled
        self.data = deepcopy(data.data)

    def position_callback(self, msg):
        self.received = True
        # print msg
        self.data.linear = deepcopy(msg)

    def orientation_callback(self, msg):
        self.received = True
        # print msg
        self.data.angular = deepcopy(msg)

    def enabled_callback(self, msg):
        # print msg
        self.enabled = True

    def calibrated_callback(self, msg):
        # print msg
        self.calibrated = True

    def gen_next(self):
        if self.mode == "positions":
            g = self.gen_positions()
            # print "gen_next: " + str(g)
            return g[1]
        else:
            g = self.gen_angles()
            print g
            return g

    def find_joint_positions(self, pose,
                             pos_x=0.0, pos_y=0.0, pos_z=0.0,
                             ori_x=0.0, ori_y=0.0, ori_z=0.0):
        '''
        Finds the joint position of the arm given some pose and the
        offsets from it
        '''
        new_pose = Point(x=pose['position'].x, # + pos_x,
                         y=pose['position'].y, # + pos_y,
                         z=pose['position'].z # + pos_z)
        )
        # print "RECEIVED ORIENTATION: " + str(pose['orientation'])
        init = tf.transformations.euler_from_quaternion(pose['orientation'])
        # angles = tf.transformations.quaternion_from_euler(
        #     pose['orientation'].x + ori_x,
        #     pose['orientation'].y + ori_y,
        #     pose['orientation'].z + ori_z
        # )
        # print "EULER ORIENTATION: " + str(init)
        print init
        angles = tf.transformations.quaternion_from_euler(
            init[0] + ori_x,
            init[1] + ori_y,
            init[2] + ori_z
        )
        new_orientation = Quaternion(x=angles[0],
                                     y=angles[1],
                                     z=angles[2],
                                     w=angles[3])
        approach_pose = Pose()
        approach_pose.position = new_pose
        approach_pose.orientation = new_orientation
        print "POSE TO BE GEN!: " + str(approach_pose)
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        self._ik_request.pose_stamp.append(pose_req)
        resp = self._iksrvp(self._ik_request)
        # print resp
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def gen_positions(self):
        if not self.enabled:
            rospy.logwarn("Myo is not enabled")
            return ("not_enabled", None)
        elif not self.calibrated:
            rospy.logwarn("Myo is not calibrated")
            return ("not_calibrated", None)
        elif self.received:
            pose_diff = self.calc_pos_off(self.data.linear)
            orient_diff = self.calc_orient_off(self.data.angular)
            px = pose_diff[0]
            py = pose_diff[1]
            pz = pose_diff[2]
            ox = orient_diff[0]
            oy = orient_diff[1]
            oz = orient_diff[2]
            new_poss = self.find_joint_positions(
                self._limb.endpoint_pose(),
                pos_x=px,
                pos_y=py,
                pos_z=pz,
                ori_x=ox,
                ori_y=oy,
                ori_z=oz)
            self.received = False
            return ("new_poss", new_poss)
        else:
            return ("Duh", None)

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

    def calc_pos_off(self, sensor_pose):
        # print "LOL"
        init_pose = []
        init_pose = [self.init_pose['position'].x,
                     self.init_pose['position'].y,
                     self.init_pose['position'].z]
        s = [sensor_pose.x,
             sensor_pose.y,
             sensor_pose.z]
        sensed_pos_diff = self._pos_diff(init_pose,
                                         s)
        # print "duh"
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
            # print p1
            # print p2
            return [x - y for (x, y) in zip(p1, p2)]

    def calc_orient_off(self, sensor_pose):
        init_pose = []
        init_pose = [self.init_pose['orientation'].x,
                     self.init_pose['orientation'].y,
                     self.init_pose['orientation'].z]
        s = [sensor_pose.x,
             sensor_pose.y,
             sensor_pose.z]
        sensed_orient_diff = self._orient_diff(init_pose,
                                               s)
        actual_orient_diff = self._orient_diff(self.init_pose,
                                               self._limb.endpoint_pose())
        diff = self._orient_diff(sensed_orient_diff, actual_orient_diff)
        # diff[2] = -diff[2]
        print "CALC'D DIFF: " + str(diff)
        return diff

    def _orient_diff(self, p1, p2):
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
        new_poss = deepcopy({'right_e0': -0.03413107249145508,
                    'right_e1': 1.5389662236877442,
                    'right_s0': 0.8149272926330567,
                    'right_s1': 0.018791264630126956,
                    'right_w0': 0.03298058690185547,
                    'right_w1': -1.5711798201965332,
                    'right_w2': -0.04793689956665039})
        if self.received:
            w0 = math.radians(self.data.angular.x)
            e1 = math.radians(self.data.angular.y)
            w1 = math.radians(self.data.angular.z)
            new_poss[self.limb_name + '_w0'] = w0 + math.radians(90.0)
            new_poss[self.limb_name + '_e1'] = e1
            new_poss[self.limb_name + '_w1'] = w1
            self.received = False
        return new_poss

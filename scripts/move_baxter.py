#!/usr/bin/python

import sys
import rospy
import rospkg
import tf
import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_tools
from copy import deepcopy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Twist,
    Quaternion,
)

from std_msgs.msg import Header

# from sensor_msgs.msg import (
#     # Image,
#     # JointState,
#     )

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class BaxterMyo(object):

    def __init__(self, limb):
        rospy.init_node("baxter_myo")
        self.name_limb = limb
        self.limb = baxter_interface.Limb(self.name_limb)
        rospy.loginfo("Enabling_Baxter...")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._rs.enable()

        rospy.Subscriber("myo_data", Twist, self.callback)
        rospy.loginfo("Subscribed to myo_data")
        self.data = Twist()

        self.received = False
        self.baxter_off = Twist()
        self.baxter_off.linear.x = 0.7523178390458416
        self.baxter_off.linear.y = -0.30058340712180104
        self.baxter_off.linear.z = -0.10224002527919201
        self.baxter_off.angular.x = 0.9327498029011391
        self.baxter_off.angular.y = 1.5512414679729603
        self.baxter_off.angular.z = 1.0006958936747292
        # self._tuck = baxter_tools.Tuck(False) # untucks
        # TODO add tucking

    def callback(self, data):
        self.received = True
        self.data = deepcopy(data)
        rospy.loginfo(rospy.get_caller_id() + " heard: \
         \n Linear [%f, %f, %f] \
         \n Angular [%f, %f, %f]", \
          self.data.linear.x, self.data.linear.y, self.data.linear.z, \
          self.data.angular.x, self.data.angular.y, self.data.angular.z)

    def find_joint_position(self, pose, x_off=0.0, y_off=0.0, z_off=0.0):
        '''
        Finds the joint position of the arm given some pose and the
        offsets from it (to avoid opening the structure all the time
        outside of the function).
        '''
        ik_srv = "ExternalTools/right/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        the_pose = deepcopy(pose)
        the_pose['position'] = Point(x=pose['position'].x + x_off,
                                     y=pose['position'].y + y_off,
                                     z=pose['position'].z + z_off)
        approach_pose = Pose()
        approach_pose.position = the_pose['position']
        approach_pose.orientation = the_pose['orientation']
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ik_request.pose_stamp.append(pose_req)
        resp = iksvc(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def find_joint_pose(self, pose, targ_x=0.0, targ_y=0.0, targ_z=0.0, targ_ox=0.0, targ_oy=0.0, targ_oz=0.0):
        '''
        Finds the joint position of the arm given some pose and the
        offsets from it (to avoid opening the structure all the time
        outside of the function).
        '''
        ik_srv = "ExternalTools/right/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        the_pose = deepcopy(pose)
        the_pose['position'] = Point(x=targ_x + self.baxter_off.linear.x,
                                     y=targ_y + self.baxter_off.linear.y,
                                     z=targ_z + self.baxter_off.linear.z)
        angles = tf.transformations.quaternion_from_euler( \
            targ_ox + self.baxter_off.angular.x, \
            targ_oy + self.baxter_off.angular.y, \
            targ_oz + self.baxter_off.angular.z)
        the_pose['orientation'] = Quaternion(x=angles[0],
                                             y=angles[1],
                                             z=angles[2],
                                             w=angles[3])
        approach_pose = Pose()
        approach_pose.position = the_pose['position']
        approach_pose.orientation = the_pose['orientation']
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ik_request.pose_stamp.append(pose_req)
        resp = iksvc(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def move_loop(self):
        i = ""
        while not rospy.is_shutdown():
            if self.received:
                new_poss = self.find_joint_pose(self.limb.endpoint_pose(),
                                               targ_x=float(self.data.linear.x),
                                               targ_y=float(self.data.linear.y),
                                               targ_z=float(self.data.linear.z),
                                               targ_ox=float(self.data.angular.x),
                                               targ_oy=float(self.data.angular.y),
                                               targ_oz=float(self.data.angular.z))
                rospy.loginfo("Position sent!")
                self.limb.move_to_joint_positions(new_poss)   # DISABLED MOTION
                self.received = False


def main():
    bm = BaxterMyo('right')
    # ang = tf.transformations.euler_from_quaternion([-0.01832673002976744, 0.7027380713920065, 0.029706873965974517, 0.7105918910470546])
    # print(ang)
    bm.move_loop()

if __name__ == "__main__":
    sys.exit(main())

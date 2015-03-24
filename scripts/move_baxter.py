#!/usr/bin/python

import sys
import rospy
import rospkg
import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_tools
# import tf
from copy import deepcopy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    # Quaternion,
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
        # self._tuck = baxter_tools.Tuck(False) # untucks
        # TODO add tucking

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

    def move_loop(self):
        i = ""
        while 1:
            try:
                x = input("x_off> ")
            except:
                x = "0.0"
            if x == "q":
                break
            try:
                y = input("y_off> ")
            except:
                y = "0.0"
            if y == "q":
                break
            try:
                z = input("z_off> ")
            except:
                z = "0.0"
            if z == "q":
                break
            new_poss = self.find_joint_position(self.limb.endpoint_pose(),
                                           x_off=float(x),
                                           y_off=float(y),
                                           z_off=float(z))
            self.limb.move_to_joint_positions(new_poss)
            print "############## Finished moving ##############"


def main():
    bm = BaxterMyo('right')
    bm.move_loop()

if __name__ == "__main__":
    sys.exit(main())

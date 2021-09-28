import math
from typing import Dict
from user import User
from uuv import UUV
import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

JOINT_NAMES = ["bravo_axis_a", "bravo_axis_b", "bravo_axis_c", "bravo_axis_d", "bravo_axis_e", "bravo_axis_f", "bravo_axis_g"]

camera_end_joint = 0
class App:
    def __init__(self):
        print("NUMPY enabled:", p.isNumpyEnabled())
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

        self.bravo_id = p.loadURDF(
                    "bpl_bravo_description/urdf/bravo_7_example_with_camera.urdf",
                    useFixedBase=True)
        joint_info = [p.getJointInfo(self.bravo_id, i)[0:2] for i in range(p.getNumJoints(self.bravo_id))]
        joint_info = [(id, name.decode("utf-8")) for id, name in joint_info]
        self.joint_indices = {str(name): id for id, name in joint_info if str(name) in JOINT_NAMES}

        default_positions = {
            "bravo_axis_a": 0,
            "bravo_axis_b": 0,
            "bravo_axis_c": math.pi * 0.,
            "bravo_axis_d": math.pi,
            "bravo_axis_e": math.pi * 0.0,
            "bravo_axis_f": math.pi * 0.5,
            "bravo_axis_g": math.pi
        }
        [p.resetJointState(self.bravo_id, jointIndex=self.joint_indices[id], targetValue=default_positions[id]) for id in JOINT_NAMES]
        self.uuv: UUV = UUV()
        self.user: User = User()

        for index, name in joint_info:
            if name == 'camera_end_joint':
                self.camera_link_id = index

            if name == 'end_effector_joint':
                self.end_effector_link = index
        return

    def run(self):

        p.addUserDebugLine([0.0, 0, 0], [0.1, 0, 0], [1, 0, 0], lineWidth=5, parentObjectUniqueId=self.bravo_id,
                           parentLinkIndex=self.camera_link_id)
        p.addUserDebugLine([0.0, 0, 0], [0, 0.1, 0], [0, 1, 0], lineWidth=5, parentObjectUniqueId=self.bravo_id,
                           parentLinkIndex=self.camera_link_id)
        p.addUserDebugLine([0.0, 0, 0], [0, 0, 0.1], [0, 0, 1], lineWidth=5, parentObjectUniqueId=self.bravo_id,
                           parentLinkIndex=self.camera_link_id)

        p.addUserDebugLine([0.0, 0, 0], [0.1, 0, 0], [1, 0, 0], lineWidth=5, parentObjectUniqueId=self.bravo_id,
                           parentLinkIndex=self.end_effector_link)
        p.addUserDebugLine([0.0, 0, 0], [0, 0.1, 0], [0, 1, 0], lineWidth=5, parentObjectUniqueId=self.bravo_id,
                           parentLinkIndex=self.end_effector_link)
        p.addUserDebugLine([0.0, 0, 0], [0, 0, 0.1], [0, 0, 1], lineWidth=5, parentObjectUniqueId=self.bravo_id,
                           parentLinkIndex=self.end_effector_link)
        while True:
            self.uuv.run()
            p.stepSimulation()
            camera_img = self.get_camera_frame()
            global_poses = self.get_global_poses_dict()
            new_pose = self.user.run(camera_img, global_poses, self.calcIK)
            [p.setJointMotorControl2(self.bravo_id, 
                jointIndex=self.joint_indices[id], 
                controlMode=p.POSITION_CONTROL,
                targetPosition=new_pose[id]) 
                for id in JOINT_NAMES]

            # cv2.imshow("View", camera_img)
            # cv2.waitKey(0)

            # get link info
            camera_state = p.getLinkState(self.bravo_id, self.camera_link_id)


            time.sleep(1./240.)
        return

    def calcIK(self, pos: np.ndarray, orient: np.ndarray = None) -> Dict[str, float]:
        jointPositions = p.calculateInverseKinematics(
            self.bravo_id,
            self.end_effector_link,
            pos,
            targetOrientation=orient
        )
        nJoints = len(JOINT_NAMES)
        return {JOINT_NAMES[i]: jointPositions[nJoints-1-i] for i in range(nJoints)}
    
    def get_global_poses_dict(self) -> Dict[str, np.ndarray]:
        end_effector_pos = p.getLinkState(
            self.bravo_id, 
            self.end_effector_link, 
            computeForwardKinematics=True
        )[4]
        camera_pos = p.getLinkState(
            self.bravo_id, 
            self.camera_link_id,
            computeForwardKinematics=True
        )[4]

        return {
            'camera_end_joint': camera_pos,
            'end_effector_joint': end_effector_pos,
        }
        
    def get_camera_frame(self):
        width, height, rgbaPixels, depthPixels, segMask = p.getCameraImage(width=640, height=480)
        img_array = np.reshape(rgbaPixels, (height, width, 4))
        # img_encoded = cv2.imencode(".jpg", img_array)
        
        return rgbaPixels
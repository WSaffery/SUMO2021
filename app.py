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
        return

    def run(self):
        while True:
            self.uuv.run()
            p.stepSimulation()
            camera_img = self.get_camera_frame()
            pose = None
            new_pose = self.user.run(camera_img, pose)
            [p.setJointMotorControl2(self.bravo_id, 
                jointIndex=self.joint_indices[id], 
                controlMode=p.POSITION_CONTROL,
                targetPosition=new_pose[id]) 
                for id in JOINT_NAMES]

            # cv2.imshow("View", camera_img)
            # cv2.waitKey(0)
            time.sleep(1./240.)
        return    

    def get_camera_frame(self):
        width, height, rgbaPixels, depthPixels, segMask = p.getCameraImage(width=640, height=480)
        img_array = np.reshape(rgbaPixels, (height, width, 4))
        # img_encoded = cv2.imencode(".jpg", img_array)
        
        return rgbaPixels
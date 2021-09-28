import math
from typing import Dict
from user import User
from uuv import UUV
import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

from scipy.spatial.transform import Rotation as R

JOINT_NAMES = ["bravo_axis_a", "bravo_axis_b", "bravo_axis_c", "bravo_axis_d", "bravo_axis_e", "bravo_axis_f", "bravo_axis_g"]

camera_end_joint = 0
class App:
    def __init__(self):
        print("NUMPY enabled:", p.isNumpyEnabled())
        self.physicsClientId = p.connect(p.GUI)
        # p.connect(p.SHARED_MEMORY)
        # self.physicsClientId = p.connect(p.DIRECT)

        # Disable additional visualisers 
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        # Set real time simulation
        p.setRealTimeSimulation(1)

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

        self.bravo_id = p.loadURDF("bpl_bravo_description/urdf/bravo_7_example_with_camera.urdf",
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
            
            pose = None
            new_pose = self.user.run(camera_img, pose)
            
            [p.setJointMotorControl2(self.bravo_id, 
                jointIndex=self.joint_indices[id], 
                controlMode=p.POSITION_CONTROL,
                targetPosition=new_pose[id]) 
                for id in JOINT_NAMES]

            cv2.imshow("View", camera_img)
            cv2.waitKey(1)
            time.sleep(1./240.)
        return    

    def get_camera_pose(self):
        pass
    
    def get_camera_frame(self):
        height = 480
        width = 640
        
        link_state = p.getLinkState(self.physicsClientId, self.camera_end_joint)
        camera_position = list(link_state[0])
        camera_euler = R.from_euler("xyz", p.getEulerFromQuaternion(link_state[1])).as_euler("zyx")
        print(camera_position, camera_euler)
        
        # ypr = R.from_quat(cameraEuler)

        # cameraQuatonian = link_state[1]
        # print(cameraQuatonian)
        
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=camera_position,
                                                          distance=0.1,
                                                          yaw=camera_euler[0],
                                                          pitch=camera_euler[1],
                                                          roll=camera_euler[2],
                                                          upAxisIndex=2)
        
        projection_matrix = p.computeProjectionMatrixFOV(fov=90, aspect=width/height, nearVal=0.1, farVal=100)
                                                
        width, height, rgbaPixels, depthPixels, segMask = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL, physicsClientId=self.physicsClientId)
        # img_array = np.reshape(rgbaPixels, (height, width, 4))
        # img_encoded = cv2.imencode(".jpg", img_array)
        
        return rgbaPixels
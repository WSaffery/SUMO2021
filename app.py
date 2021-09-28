import math
import time
import cv2
import numpy as np
import argparse

import pybullet as p
import pybullet_data

from typing import Dict

from scipy.spatial.transform import Rotation as R

from user import User
from uuv import UUV


JOINT_NAMES = ["bravo_axis_a", "bravo_axis_b", "bravo_axis_c", "bravo_axis_d", "bravo_axis_e", "bravo_axis_f", "bravo_axis_g"]

SHOW_JAW_PROJECTION = 0.1
HIDE_JAW_PROJECTION = 0.001


class App:
    height = 480
    width = 640
    
    projection_matrix = p.computeProjectionMatrixFOV(fov=100, aspect=width/height, nearVal=SHOW_JAW_PROJECTION, farVal=3.5)
    
    T_cb = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 1.]])
    
    def __init__(self):
        parser = self.init_argparse()
        args = parser.parse_args()
        if not args.round:
            raise argparse.ArgumentError(args.round, "Round argument (-r/--round) not provided.")

        self.round = args.round

        print("NUMPY enabled:", p.isNumpyEnabled())
        
        self.physicsClientId = p.connect(p.GUI)

        # Disable additional visualiser controls 
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)

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
            "bravo_axis_g": math.pi * 0.5
        }
        [p.resetJointState(self.bravo_id, jointIndex=self.joint_indices[id], targetValue=default_positions[id]) for id in JOINT_NAMES]
        self.uuv: UUV = UUV(int(self.round))
        self.user: User = User()
        self.ticks = 0

        for index, name in joint_info:
            if name == 'camera_end_joint':
                self.camera_link_id = index

            if name == 'end_effector_joint':
                self.end_effector_link = index
        return
    
    def init_argparse(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser(
            usage="%(prog)s",
            description="BLUEPRINT LAB SUMO HACKATHON CHALLENGE 2021"
        )
        parser.add_argument("-r", "--round", choices=['1', '2', '3'])
        return parser

    def run(self):
        start_time = time.time()  # Start time in seconds 
        
        while True:
            self.uuv.run(self.ticks)
            p.stepSimulation()
            self.ticks += 1

            camera_img = self.get_camera_frame()
            global_poses = self.get_global_poses_dict()
            
            time_remaining = 59.0 - (time.time() - start_time)
            text = f'Round: {self.round}    Time remaining: {round(time_remaining, 2)}'
            
            if time_remaining > 0.0:
                cv2.putText(camera_img, text, (10, self.height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(camera_img, text, (10, self.height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
            
            new_pose = self.user.run(camera_img, global_poses, self.calcIK)
            [p.setJointMotorControl2(self.bravo_id, 
                jointIndex=self.joint_indices[id], 
                controlMode=p.POSITION_CONTROL,
                targetPosition=new_pose[id]) 
                for id in JOINT_NAMES]
            
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
    
    def get_view_matrix(self):
        # Get the current wrist camera frame
        camera_frame = p.getLinkState(self.physicsClientId, self.camera_link_id)

        # Extract wrist camera frame translation vector and rotatin matrix
        t_cb = np.array(camera_frame[0]).T
        R_cb = R.from_quat(camera_frame[1]).as_matrix()
        
        # Compute homogeious trnasformation between base and camera frame
        self.T_cb[0:3, 0:3] = R_cb  # Set rotation matrix
        self.T_cb[0:3, -1] = t_cb  # Set translation
        T_bc = np.linalg.inv(self.T_cb)  # Get inverse

        # Extract view matrix from transformation (column major format), and return
        return T_bc.T.flatten()
    
    def get_camera_frame(self):
        view_matrix = self.get_view_matrix()                                      
        width, height, rgbaPixels, depthPixels, segMask = p.getCameraImage(self.width, self.height, 
                                                                           view_matrix, self.projection_matrix, 
                                                                           renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgbaPixels = cv2.cvtColor(rgbaPixels, cv2.COLOR_BGR2RGB)
        
        return rgbaPixels
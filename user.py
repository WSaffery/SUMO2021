""" User code lives here """
import time
from typing import Dict
import math
from typing import Callable, Optional
import numpy as np
import cv2
from enum import Enum
import pybullet as p

height = 480
width = 640

fmco=height/(2 * math.tan((100/2) * (math.pi / 180)))
matrix_coefficients = np.array([[fmco,0,width/2],[0,fmco,height/2],[0,0,1]])

RoboStates = Enum('RoboStates', 'Searching Located_1 Located_2 Grabbing')

use3D = True

class User:
    def __init__(self) -> None:
        self.pose = {
            "bravo_axis_a": 0.05,
            "bravo_axis_b": 0,
            "bravo_axis_c": math.pi * 0.5,
            "bravo_axis_d": math.pi * 0,
            "bravo_axis_e": math.pi * 0.75,
            "bravo_axis_f": math.pi * 0.9,
            "bravo_axis_g": math.pi
        }
        self.vec = [0,0,0]
        self.quat = [0,0,0,0]
        self.targets = {}
        self.targetLastUpdate = {}
        self.state = RoboStates.Searching
        self.grabTarget = []
        self.visualProps = {}
        return

    def updateProp(self, name, vec3):
        if use3D:
            colours = ["R", "G", "B"]
            colour = colours[name]
            if name in self.visualProps:
                p.resetBasePositionAndOrientation(self.visualProps[name], vec3, p.getQuaternionFromEuler([0,math.pi, 0]))
            else:
                self.visualProps[name]: int = p.loadURDF(f"./sphere{colour}.urdf", basePosition = vec3)

    def quaternionRotationMatrix(self, Q): # compressed jay's implementation
        return np.array([[2 * (Q[0] * Q[0] + Q[1] * Q[1]) - 1, 2 * (Q[1] * Q[2] - Q[0] * Q[3]), 2 * (Q[1] * Q[3] + Q[0] * Q[2])],
                        [2 * (Q[1] * Q[2] + Q[0] * Q[3]), 2 * (Q[0] * Q[0] + Q[2] * Q[2]) + 1, 2 * (Q[2] * Q[3] - Q[0] * Q[1])],
                        [2 * (Q[1] * Q[3] - Q[0] * Q[2]), 2 * (Q[2] * Q[3] + Q[0] * Q[1]), 2 * (Q[0] * Q[0] + Q[3] * Q[3]) - 1]])

    def inverseCameraProjection(self, camera_pos, camera_angle, pos):
        return -np.matmul(self.quaternionRotationMatrix(camera_angle), pos) + camera_pos

    def setPose(self, calcIK, vec, quat):
        self.vec = vec
        self.quat = quat
        self.pose = calcIK(vec, quat)

    def run(self, image: list,  global_poses: Dict[str, np.ndarray], calcIK: Callable[[np.ndarray, Optional[np.ndarray]], Dict[str, float]]) -> Dict[str, float]:

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        tags = []
        if corners:
            for tag,id in zip(corners, ids):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(tag, 0.06, matrix_coefficients, distCoeffs=None)
                self.targets[id[0]] = self.inverseCameraProjection(global_poses["camera_end_joint"][0], global_poses["camera_end_joint"][1], tvec[0][0])
                self.targetLastUpdate[id[0]] = time.time()
                self.updateProp(id[0], self.targets[id[0]])
                tags.append([tvec[0][0], id[0]])
                cv2.aruco.drawAxis(image, matrix_coefficients, None, rvec, tvec, 0.03)

        n_tags = len(self.targets)
        image = cv2.putText(image, self.state.name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))

        if tags:
            if (n_tags==1):
                self.state = RoboStates.Located_1
            elif (n_tags==2):
                self.state = RoboStates.Located_2

        if self.state == RoboStates.Searching:
            on_line_pos = np.array([0.65, 0.15, -0.05]) + math.sin(time.time()) * np.array([0.3, 0.3, 0])
            self.setPose(calcIK, on_line_pos, p.getQuaternionFromEuler([0, math.pi/2, 0]))

        if self.state == RoboStates.Located_1:
            if 0 in self.targets and ((time.time()-self.targetLastUpdate[0])>0.3):
                 del self.targets[0]
            if 1 in self.targets and ((time.time()-self.targetLastUpdate[1])>0.3):
                 del self.targets[1]
            if 0 in self.targets:
                target_pos = self.targets[0]
                self.grabTarget = (target_pos[0]+0.15,target_pos[1],target_pos[2])
                self.state = RoboStates.Grabbing
                self.enteredGrabbing = time.time()
            if 1 in self.targets:
                target_pos = self.targets[1]
                self.grabTarget = (target_pos[0]-0.15,target_pos[1],target_pos[2])
                self.state = RoboStates.Grabbing
                self.enteredGrabbing = time.time()

        if self.state == RoboStates.Located_2:
            if ((time.time()-self.targetLastUpdate[0])>0.3):
                 del self.targets[0]
            if ((time.time()-self.targetLastUpdate[1])>0.3):
                 del self.targets[1]
            n_tags = len(self.targets)
            if (n_tags==1):
                self.state = RoboStates.Located_1
            elif (n_tags==0):
                self.state = RoboStates.Searching
            else:
                self.grabTarget = (self.targets[0]+self.targets[1])/2
                self.state = RoboStates.Grabbing
                self.enteredGrabbing = time.time()

        if self.state == RoboStates.Grabbing:
            # If we're at the vec position, go back to searching
            if (time.time()-self.enteredGrabbing)>2:
                self.state = RoboStates.Searching
                self.targets = {}
            else:
                old = {k:v for k,v in self.pose.items()}
                self.setPose(calcIK, self.grabTarget+np.array([math.sin(time.time()*3)*0.01,math.sin(time.time()*3)*0.01,0]), None)
                compare = True
                for k in old.keys():
                    if round(old[k],3) != round(self.pose[k],3):
                        compare = False
                        break
                if compare:
                    self.state = RoboStates.Searching
                    self.targets = {}

        cv2.imshow("View", image)
        cv2.waitKey(1)
        return self.pose

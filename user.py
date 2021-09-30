""" User code lives here """
import time
from typing import Dict
import math
from typing import Callable, Optional
import numpy as np
import cv2
from enum import Enum
import pybullet as p
# import asyncio

height = 480
width = 640

fmco=height/(2 * math.tan((100/2) * (math.pi / 180)))
matrix_coefficients = np.array([[fmco,0,width/2],[0,fmco,height/2],[0,0,1]])
distortion_coefficients = np.array([0.,0.,0.,0.,0.])

RoboStates = Enum('RoboStates', 'Searching Located_1 Located_2 Grabbing')

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
        self.inc = 0.1
        self.last_time = time.time()
        self.vec = [0,0,0]
        self.quat = [0,0,0,0]
        self.roam_default = (np.array([1.4, 0, 1.2]), p.getQuaternionFromEuler([0,math.pi/2,0]))
        self.targets = {}
        self.visualProps = {}
        self.state = RoboStates.Searching
        self.searchState = {"Mode": 0, "Val": 2}
        self.grabTarget = []
        return

    def updateProp(self, name, vec3):
        colours = ["R", "G", "B"]
        colour = colours[name]
        if name in self.visualProps:
            p.resetBasePositionAndOrientation(self.visualProps[name], vec3, p.getQuaternionFromEuler([0,math.pi, 0]))
        else:
            self.visualProps[name]: int = p.loadURDF(f"./sphere{colour}.urdf", basePosition = vec3)

    def quaternionRotationMatrix(self, Q):
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) + 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])
        return rot_matrix

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
                self.updateProp(id[0], self.targets[id[0]])
                tags.append([tvec[0][0], id[0]])
                cv2.aruco.drawAxis(image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.03)

        n_tags = len(self.targets)
        image = cv2.putText(image, self.state.name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
        # the actual machine
        #Hey buddy

        if tags:
            if (n_tags==1):
                self.state = RoboStates.Located_1
            elif (n_tags==2):
                self.state = RoboStates.Located_2

        if self.state == RoboStates.Searching:
            pos, orient = self.roam_default
            modes = [(0, self.searchState["Val"]), (self.searchState["Val"],0), (0, -self.searchState["Val"]), (-self.searchState["Val"], 0)]
            if self.searchState["Mode"] == 0:
                self.searchState["Val"] += 4
            x, y =  modes[self.searchState["Mode"]]
            pos = (pos[0]+x, pos[1]+y, pos[2])
            self.searchState["Mode"] = (self.searchState["Mode"] + 1)%4
            self.setPose(calcIK, pos, orient)
            time.sleep(0.01)

        if self.state == RoboStates.Located_1:
            target_id, target_pos = list(self.targets.items())[0]
            print(f"{target_id=} {target_pos=}")
            unsigned_offset = 0.15
            offset = -unsigned_offset if target_id == 1 else unsigned_offset
            self.grabTarget = (target_pos[0]+offset,target_pos[1],target_pos[2])
            self.state = RoboStates.Grabbing

        if self.state == RoboStates.Located_2:
            print(f"{self.targets[0]=}")
            print(f"{self.targets[1]=}")
            self.grabTarget = (self.targets[0]+self.targets[1])/2
            self.state = RoboStates.Grabbing

        if self.state == RoboStates.Grabbing:
            old = {k:v for k,v in self.pose.items()}
            print(f"middle {self.grabTarget=}")
            self.setPose(calcIK, self.grabTarget, p.getQuaternionFromEuler([0,math.pi/2,0]))
            compare = True
            for k in old.keys():
                if round(old[k],3) != round(self.pose[k],3):
                    compare = False
                    break
            if compare:
                self.state = RoboStates.Searching
                # Resets so it doesn't average between old and new points after a static break (moved UAV)
                # Experimental
                self.targets = {}


        cv2.imshow("View", image)
        cv2.waitKey(1)
        return self.pose

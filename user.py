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
        self.roam_default = (np.array([0.6, 0, 7]), p.getQuaternionFromEuler([0,math.pi/2,0]))
        self.targets = {}
        self.targetLastUpdate = {}
        self.visualProps = {}
        self.state = RoboStates.Searching
        self.searchState = {"Mode": 0, "Val": 0.2}
        self.grabTarget = []
        return

    def updateProp(self, name, vec3):
        colours = ["R", "G", "B"]
        colour = colours[name]
        if name in self.visualProps:
            pass
            #p.resetBasePositionAndOrientation(self.visualProps[name], vec3, p.getQuaternionFromEuler([0,math.pi, 0]))
        else:
            pass
            #self.visualProps[name]: int = p.loadURDF(f"./sphere{colour}.urdf", basePosition = vec3)
        self.targetLastUpdate[name] = time.time()

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

    def locateBlob(self, image):
        lower_white = np.array([220,220, 220])
        upper_white = np.array([255, 255, 255])
        # Threshold the HSV image
        mask = cv2.inRange(image, lower_white, upper_white)
        # Remove noise
        kernel_erode = np.ones((4,4), np.uint8)
        eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
        kernel_dilate = np.ones((6,6),np.uint8)
        dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
        # Find the different contours
        contours, hierarchy = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
        if len(contours) > 0:
            M = cv2.moments(contours[0])
            # Centroid
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            return cx, cy
        else:
            return None

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
                cv2.aruco.drawAxis(image, matrix_coefficients, None, rvec, tvec, 0.03)

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
            on_line_pos = np.array([0.65, 0.15, -0.05]) + math.sin(time.time()) * np.array([0.3, 0.3, 0])
            self.setPose(calcIK, on_line_pos, p.getQuaternionFromEuler([0, math.pi/2, 0]))
            # self.setPose(calcIK, np.array([math.sin(time.time())*0.3, 0, 0.5]), p.getQuaternionFromEuler([math.sin(time.time())*0.3,math.pi/4,math.sin(time.time())*0.3]))
            # pos, orient = self.roam_default
            # modes = [(-self.searchState["Val"], self.searchState["Val"]), (self.searchState["Val"],self.searchState["Val"]), (self.searchState["Val"], -self.searchState["Val"]), (-self.searchState["Val"], -self.searchState["Val"])]
            # x, y =  modes[self.searchState["Mode"]]
            # pos = (pos[0]+x, pos[1]+y, pos[2])
            # self.searchState["Mode"] = (self.searchState["Mode"] + 1)%4
            # if self.searchState["Mode"] == 0:
            #     self.searchState["Val"] += 0.1
            # self.setPose(calcIK, pos, orient)

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
            if (time.time()-self.enteredGrabbing)>2:
                self.state = RoboStates.Searching
                self.targets = {}
            else:
                old = {k:v for k,v in self.pose.items()}
                print(f"middle {self.grabTarget=}")
                self.setPose(calcIK, self.grabTarget+np.array([math.sin(time.time()*3)*0.01,math.sin(time.time()*3)*0.01,0]), None)

                # If we're at the vec position, go back to searching
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

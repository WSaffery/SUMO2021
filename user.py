""" User code lives here """
import time
from typing import Dict
import math
from typing import Callable, Optional
import numpy as np
import cv2
from pupil_apriltags import Detector
from random import random
import pybullet as p

SHOW_JAW_PROJECTION = 0.001
height = 480
width = 640
ADJUST = 136.25

print("matrix stuff")
# computed = p.computeProjectionMatrixFOV(fov=100, aspect=width/height, nearVal=SHOW_JAW_PROJECTION, farVal=3.5)
# changed = np.array(computed).reshape(4, 4)
# view_matrix = np.array([8.90742484e-01,7.29864445e-04,-4.54507749e-01, -1.28688351e-06,  9.99998715e-01,  1.60331089e-03, 4.54508335e-01, -1.42755223e-03,  8.90741340e-01, -5.80138836e-01,  5.22229923e-02, -1.70142163e-01]).reshape(4, 3)
# changed = np.matmul(changed, view_matrix).T
# (matrix_coefficients, distortion_coefficients, _, _, _, _, _) = cv2.decomposeProjectionMatrix(changed)
# 16 elements

#print(computed)
matrix_coefficients = np.load("calibration_matrix.npy")#np.array(computed)
# 3x3
#print(matrix_coefficients)
distortion_coefficients = np.load("distortion_coefficients.npy")
# 5x1
#print(distortion_coefficients)
#distortion_coefficients = np.array([computed[0][0], computed[0][5], computed[0][10], computed[0][11], computed[0][13]])
#[[ 0.02206642  0.20438685 -0.00633739 -0.00140045 -0.85132748]]
print(distortion_coefficients)

class Tag:
    def __init__(self, center, rvec, id):
        self.center = center
        self.rvec = rvec
        self.id = id

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
        self.oldpose = self.pose
        self.inc = 0.1
        self.last_time = time.time()
        self.locking = False
        self.target_pos = self.target_orient = None
        self.targets = {}
        self.moving = 0
        self.default = [(0.5211272239685059, 8.080899533524644e-06, 0.22556839883327484), (0.0001061355578713119, 0.5224985480308533, -1.2371250704745762e-05, 0.8526401519775391)]
        # self.roam_default = (np.array([0.5, 0, 0.5]), p.getQuaternionFromEuler([0,math.pi/2,0]))
        self.roam_default = (np.array([1.2, 0, 0.8]), p.getQuaternionFromEuler([0,math.pi/2,0]))
        self.roam = self.roam_default
        return

    def manual_control(self, global_poses, calcIK):
        lineIn = input("input command: ")
        commandList = lineIn.split(" ")
        command = commandList[0]
        args = commandList[1:]
        if command == "data":
            print(f"{global_poses['camera_end_joint']=}")
            print(f"{global_poses['end_effector_joint']=}")
            print(f"{self.pose=}")
            print(f"{self.inc=}")
        if command == "pq":
            vec3 = np.array([float(x) for x in args[0:3]])
            ori = np.array([float(x) for x in args[3:7]])
            print(vec3, ori)
            self.pose = calcIK(vec3, ori)
        elif command == "p":
            vec3 = np.array([float(x) for x in args[0:3]])
            ori = None
            print(vec3, ori)
            self.pose = calcIK(vec3, ori)
        elif command == "p":
            vec3 = np.array([float(x) for x in args[0:3]])
            inc = args[3]
            print(vec3, inc)
            self.pose = calcIK(vec3, None)
            self.inc = inc
        elif command == "inc":
            self.inc = args[0]
        elif command == "camera":
            global_poses['camera_end_joint'] = args[0]
        elif command == "end":
            global_poses['end_effector_joint'] = args[0]

    def averagePerVal(listA, listB):
        return  [(a+b)/2 for a,b in zip(listA, listB)]

    def percentChange(listA, listB):
        return [1 + (a - b)/b for a,b in zip(listA, listB)]

    def percentApply(list, percents):
        return [n*p for n,p in zip(list, percents)]

    def Algo(global_poses, x, y, z):
        current = global_poses['end_effector_joint'][0]
        current_x, current_y, current_z = current
        # print(f"current {current_x=} {current_y=} {current_z=}")
        to_x = current_x + x*0.1
        to_y = current_y + y*0.1
        to_z = current_z + z*0.1
        return np.array([to_x,to_y,to_z])

    def Solvo(global_poses, x, y, z):
        # default_camera_pos = (0.4393743574619293, -0.051950227469205856, 0.4152250289916992)
        camera_pos, camera_angle = global_poses["camera_end_joint"]
        arm_pos, arm_angle = global_poses["end_effector_joint"]
        relative_pos_percent = User.percentChange(arm_pos, camera_pos)
        # relative_angle_percent = User.percentChange(arm_angl, camera_angle)
        relative_pos = [a+b for a,b in zip(camera_pos, (x,y,z))]
        proper_pos = User.percentApply(relative_pos, relative_pos_percent)
        proper_angle = arm_angle
        return np.array(proper_pos), proper_angle

    def moveTo3D(self, calcIK, x, y, z):
        vec3 = (x, y, z)
        joints = calcIK(vec3, None)
        # print(f"moveTo vector {vec3=}")
        # print(joints := calcIK(vec3, None))
        self.pose = joints

    def relativeToArm(global_poses, pos, orient):
        camera_pos, camera_angle = global_poses["camera_end_joint"]
        arm_pos, arm_angle = global_poses["end_effector_joint"]
        relative_pos_percent = User.percentChange(camera_pos, arm_pos)
        relative_angle_percent = User.percentChange(camera_angle, arm_angle)
        return relative_pos_percent, relative_angle_percent

    def setTargets3D(self, tags, global_poses):
        for t in tags:
            t.absPos = User.Solvo(global_poses, *t.center)
            self.targets[t.id] = t

        if len(self.targets) == 2:
            if self.targets[0].absPos[1] == self.targets[1].absPos[1]:
                self.target_pos = User.averagePerVal(self.targets[0].absPos[0], self.targets[1].absPos[0])
                self.target_orient = self.targets[0].absPos[1]
            else:
                # To be updated to take into account the impact of angles
                self.target_pos = User.averagePerVal(self.targets[0].absPos[0], self.targets[1].absPos[0])
                self.target_orient =  User.averagePerVal(self.targets[0].absPos[1], self.targets[1].absPos[1])
        else:
            tag = tags[0]
            id = tag.id
            self.target_pos = tag.absPos[0]
            self.target_orient = tag.absPos[1]
        # print(f"set targets {self.target_x=} {self.target_y=} {self.target_z=}")

    def toProperList(tvec):
        out = tvec.tolist()[0][0]
        # print(out)
        return out

    def run(self,
            image: list,
            global_poses: Dict[str, np.ndarray],
            calcIK: Callable[[np.ndarray, Optional[np.ndarray]], Dict[str, float]],
            ) -> Dict[str, float]:
        """Run loop to control the Bravo manipulator.

        Parameters
        ----------
        image: list
            The latest camera image frame.

        global_poses: Dict[str, np.ndarray]
            A dictionary with the global camera and end-effector pose. The keys are
            'camera_end_joint' and 'end_effector_joint'. Each pose consitst of a (3x1)
            position (vec3) and (4x1) quaternion defining the orientation.

        calcIK: function, (pos: np.ndarray, orient: np.ndarray = None) -> Dict[str, float]
            Function to calculate inverse kinematics. Provide a desired end-effector
            position (vec3) and an orientation (quaternion) and it will return a pose
            dictionary of joint angles to approximate the pose.
        """
        global_orients = [v[1] for v in global_poses.values()]
        print(global_orients)
        # print(User.averagePerVal(global_orients[0], global_orients[1]))
        # 'camera_end_joint': [(0.4393743574619293, -0.051950227469205856, 0.4152250289916992), (-0.0007773424149490893, -0.23344528675079346, -0.0001872739812824875, 0.9723696112632751)],

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
        if not ids is None:
            ids = ids.tolist()[0]
        cv2.aruco.drawDetectedMarkers(image, corners)
        cv2.aruco.drawDetectedMarkers(image, rejected)
        # print(ids)

        tags = []
        if corners and not ids is None:
            for tag,id in zip(corners, ids):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.02, matrix_coefficients, distortion_coefficients)
                tags.append(Tag(User.toProperList(tvec), rvec, id))
                cv2.aruco.drawAxis(image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
                # print(f"TAG{id}")
                # print(rvec)
                # print(tvec)

        # print(f"{tags=}")
        if tags:
            self.setTargets3D(tags, global_poses)
            print(f"Moving guided to {self.target_pos=} {self.target_orient=}")
            self.pose = calcIK(self.target_pos, self.target_orient)
            # self.pose = calcIK(self.target_pos, global_poses['camera_end_joint'][1])
            # self.moveTo3D(calcIK, self.target_x, self.target_y, self.target_z)
            self.locking = True
        elif not self.locking:
            # print(f"hello {global_poses['end_effector_joint']}")
            pos, orient = self.roam
            print(f"Moving at random to {pos=}")
            self.pose = calcIK(pos, orient)
            # self.pose = self.default
            pos = [pos[0]+(random()-0.5)*0.5, pos[1]+(random()-0.5)*0.5, pos[2]]
            # pos = [x+(random.random()-0.5)*0.5 for x in pos]
            # orient = [x+(random()-random())*2 for x in orient]
            self.roam = [pos,orient]
            self.moving -= 1
            if (self.moving == 0):
                self.moving = 5
                self.roam = self.roam_default
        elif not self.target_pos is None and not self.target_orient is None:
            print(f"continuing without sight to {self.target_pos=} {self.target_orient=}")
            self.pose = calcIK(self.target_pos, self.target_orient)
            # self.pose = calcIK(self.target_pos, global_poses['camera_end_joint'][1])
            # self.moveTo3D(calcIK, self.target_x, self.target_y, self.target_z)
            self.moving += 1
            if (self.moving > 20):
                self.locking = False

        # end info output

        cv2.imshow("View", image)
        cv2.waitKey(1)


        return self.pose

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
fmco=height/(2 * math.tan((100/2) * (math.pi / 180)))
# fmco = 1 / math.tan((100/2)*(math.pi/180))
# fmco = height / 2*math.tan((100/2)*(math.pi/180))
matrix_coefficients = np.array([fmco,0,width/2,0,fmco,height/2,0,0,1]).reshape((3, 3))
distortion_coefficients = np.array([0.,0.,0.,0.,0.])

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
        self.all_targets = {0:[], 1:[]}
        self.targets = {}
        self.moving = 0
        self.default = [(0.5211272239685059, 8.080899533524644e-06, 0.22556839883327484), (0.0001061355578713119, 0.5224985480308533, -1.2371250704745762e-05, 0.8526401519775391)]
        # self.roam_default = (np.array([0.5, 0, 0.5]), p.getQuaternionFromEuler([0,math.pi/2,0]))
        self.roam_default = (np.array([1.4, 0, 1.2]), p.getQuaternionFromEuler([0,math.pi/2,0]))
        self.roam = self.roam_default
        self.locked = 0
        self.mode = 0
        self.lockedin = 0
        self.visualProps = {}
        return

    def updateProp(self, name, vec3):
        colours = ["R", "G", "B"]
        colour = colours[name]
        if name in self.visualProps:
            p.resetBasePositionAndOrientation(self.visualProps[name], vec3, p.getQuaternionFromEuler([0,math.pi, 0]))
        else:
            self.visualProps[name]: int = p.loadURDF(f"./sphere{colour}.urdf", basePosition = vec3)

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

    def TargetsLen(targets):
        return min(1,len(targets[0])) + min(1,len(targets[1]))

    def averagePerVal(listA, listB):
        return  [(a+b)/2 for a,b in zip(listA, listB)]

    def averagePerValManyList(lists):
        # print(lists)
        # print({*lists})
        # print(list(zip(*lists)))
        # print(list(zip(*(lists[0])))})
        return  [sum(x)/len(x) for x in zip(*(lists))]

    def percentChange(listA, listB):
        return [1 + (a - b)/b for a,b in zip(listA, listB)]

    def percentApply(list, percents):
        return [n*p for n,p in zip(list, percents)]

    def quaternion_rotation_matrix(Q):
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


    def Solvo(global_poses, x, y, z):
        camera_pos, camera_angle = global_poses["camera_end_joint"]
        cam_rot_matrix = User.quaternion_rotation_matrix(camera_angle)
        cam_pos_vector = np.array(camera_pos)
        tag_pos_vector = np.array((x,y,z))
        # p.loadURDF("./sphereB.urdf", basePosition = (x,y,z))
        print(f"{cam_pos_vector=} {np.matmul(cam_rot_matrix, tag_pos_vector)=}")
        temp = np.matmul(cam_rot_matrix, tag_pos_vector)
        absolute_point =  cam_pos_vector - temp
        arm_pos, arm_angle = global_poses["end_effector_joint"]
        # p.loadURDF("./sphereR.urdf", basePosition = absolute_point)
        proper_angle = arm_angle
        return np.array(absolute_point), proper_angle

    def simpleSolvo(global_poses, tvec):
        camera_pos, camera_angle = global_poses["camera_end_joint"]
        simplePos = camera_pos - tvec[0][0]
        # p.loadURDF("./sphereR.urdf", basePosition = simplePos)
        return simplePos


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

    def updateTargets(self, id):
        absPosList = [x.absPos for x in self.all_targets[id]]
        posList = []
        orientList = []
        for x in absPosList:
            posList.append(x[0])
            orientList.append(x[1])
        pos = User.averagePerValManyList(posList)
        orient = User.averagePerValManyList(orientList)
        self.targets[id].absPos = (pos, orient)
        # print(f"{sum(pos)=} {sum(self.targets[id].absPos[0])=}")
        # print(abs((sum(pos)-sum(self.targets[id].absPos[0])/len(pos))))
        # if (abs((sum(pos)-sum(self.targets[id].absPos[0])/len(pos))) <= 0.5):
        #     print("infavour of collective")
        #     self.targets[id].absPos = (pos, orient)
        # else:
        #     print("drastic shift")
        #     self.all_targets = {0:[],1:[]}
        #     self.all_targets[id].append(self.targets[id])

    def setTargets3D(self, tags, global_poses):
        for t in tags:
            t.absPos = User.Solvo(global_poses, *t.center)
            self.all_targets[t.id].append(t)
            self.targets[t.id] = t
            # if len(self.all_targets[t.id]) > 1:
            #     self.updateTargets(t.id)

        if len(self.targets) == 2:
            self.updateProp(0, self.targets[0].absPos[0])
            self.updateProp(1, self.targets[1].absPos[0])
            print("Two spotted")
            if self.targets[0].absPos[1] == self.targets[1].absPos[1]:
                print("Same orientation spot")
                self.target_pos = User.averagePerVal(self.targets[0].absPos[0], self.targets[1].absPos[0])
                self.target_orient = self.targets[0].absPos[1]
                print(f"{self.targets[0].absPos[0]=},{self.targets[1].absPos[0]=}")
            else:
                # To be updated to take into account the impact of angles more accurately
                print("Patchwork spot")
                self.target_pos = User.averagePerVal(self.targets[0].absPos[0], self.targets[1].absPos[0])
                self.target_orient = User.averagePerVal(self.targets[0].absPos[1], self.targets[1].absPos[1])
                # self.target_orient =  User.averagePerVal(self.targets[0].absPos[1], self.targets[1].absPos[1])
                print(f"{self.targets[0].absPos[0]=},{self.targets[1].absPos[0]=}")
                print(f"{self.targets[1].absPos[1]=},{self.targets[1].absPos[1]=}")
        else:
            tag = tags[0]
            self.updateProp(tag.id, tag.absPos[0])
            self.target_pos = tag.absPos[0]
            self.target_orient = tag.absPos[1]
        # print(f"set targets {self.target_x=} {self.target_y=} {self.target_z=}")

    def modedPos(self, arm_pos):
        moded_pos = [x for x in arm_pos]
        moded_pos[self.mode] = self.target_pos[self.mode]
        return moded_pos

    def updateMode(self):
        self.mode = (self.mode + 1)%3

    def toProperList(tvec):
        out = tvec.tolist()[0][0]
        # print(out)
        return out

    class Search:
        Mode = 0
        Val = 2
        num_points = 8
        points = []
        def rotate(x, y, r):
            rx = (x*math.cos(r)) - (y*math.sin(r))
            ry = (y*math.cos(r)) + (x*math.sin(r))
            return (rx, ry)

        def point_ring(center, point_num):
            radius = User.Search.Val/16
            arc = (2 * math.pi) / User.Search.num_points # what is the angle between two of the points
            if len(User.Search.points) <= point_num:
                (px,py) = User.Search.rotate(0, radius, arc * point_num)
                px += center[0]
                py += center[1]
            return (px,py)

        def search_movement(user, coordinates=None):
            if coordinates is None:
                coordinates=user.roam_default
            pos, orient = coordinates
            x, y = User.Search.point_ring(pos, User.Search.Mode)
            pos = x,y,pos[2]
            User.Search.Mode = (User.Search.Mode + 1)%User.Search.num_points
            p.loadURDF("./sphereRR.urdf", basePosition = pos)
            print("HIHIHIHIHIHIHIHIHIHIHIHIHIHIHI")
            # p.loadURDF(f"./sphere{['R','G','B'][User.Search.Mode%3]}.urdf", basePosition = pos)
            return pos, orient

        # def search_movement(user, coordinates=None):
        #     if coordinates is None:
        #         coordinates=user.roam_default
        #     pos, orient = coordinates
        #     x, y = User.Search.get_XY()
        #     pos = (pos[0]+x, pos[1]+y, pos[2])
        #     User.Search.Mode = (User.Search.Mode + 1)%4
        #     return pos, orient
        #
        # def get_XY():
        #     modes = [(0, User.Search.Val), (User.Search.Val,0), (0, -User.Search.Val), (-User.Search.Val, 0)]
        #     if User.Search.Mode == 0:
        #         User.Search.Val += 4
        #     return modes[User.Search.Mode]

    def single_target(self):
        target = self.targets[0]
        pos, orient = target.absPos
        if target.id == 1:
            pos[0] -= 0.3
        else:
            pos[0] += 0.3
        return pos, orient

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
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
        if not ids is None:
            ids = ids.tolist()[0]
        cv2.aruco.drawDetectedMarkers(image, corners)
        cv2.aruco.drawDetectedMarkers(image, rejected)

        tags = []
        if corners and not ids is None:
            for tag,id in zip(corners, ids):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.06, matrix_coefficients, distortion_coefficients)
                tags.append(Tag(User.toProperList(tvec), rvec, id))
                cv2.aruco.drawAxis(image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


        if tags:
            self.setTargets3D(tags, global_poses)

        if len(self.targets) == 2:
            self.locking = True
            print(f"Moving guided 2x to {self.target_pos=} {self.target_orient=}")
            old = {k:v for k,v in self.pose.items()}
            val = 50
            self.updateProp(2, self.target_pos)
            if self.lockedin == 0:
                self.pose = calcIK(self.target_pos, self.target_orient)
                compare = True
                for k in old.keys():
                    if round(old[k],3) != round(self.pose[k],3):
                        compare = False
                        break
                if compare:
                    self.lockedin = val
                    self.target_pos[2] -= 0.2
                    self.pose = calcIK(self.target_pos, self.target_orient)
            else:
                self.all_targets = {0:[],1:[]}
                x, y, z = global_poses["end_effector_joint"][0]
                z -= 0.2
                print(f"Move down {(x,y,x)=}")
                self.pose = calcIK((x,y,z),  p.getQuaternionFromEuler([0,math.pi/2,0]))
                self.lockedin -= 1

        elif len(self.targets) == 1 and self.moving < 5:
            print(f"Moving guided 1x to {self.target_pos=} {self.target_orient=}")
            self.pose = calcIK(*self.single_target())
            self.moving += 1
            if (self.moving > 5):
                self.locking = False
        elif not self.locking:
            # pos, orient = User.Search.search_movement(self, self.single_target())
            if len(self.targets) == 1:
                pos, orient = User.Search.search_movement(self, self.single_target())
            else:
                pos, orient = User.Search.search_movement(self)
            print(f"Moving unguided search to {pos=} {orient=}")
            self.pose = calcIK(pos, orient)
            time.sleep(0.2)


        # end info output

        cv2.imshow("View", image)
        cv2.waitKey(1)


        return self.pose

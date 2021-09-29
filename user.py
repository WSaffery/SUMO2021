""" User code lives here """
import time
from typing import Dict
import math
from typing import Callable, Optional
import numpy as np
import cv2
from pupil_apriltags import Detector
from random import random

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
        self.target_x = self.target_y = None
        self.moving = 0
        self.default = [0.5, 0, 0]
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


    def flatAlgo(global_poses, x, y):
        current = global_poses['end_effector_joint'][0]
        current_x, current_y, current_z = current
        print(f"current {current_x=} {current_y=} {current_z=}")
        to_x = current_x + ((x-320)/640)*0.1
        to_y = current_y + ((y-240)/480)*0.1
        if (to_x != 0 or to_y != 0):
            to_z = 0
        else:
            to_z = current_z - 0.1
        return np.array([to_x,to_y,to_z])

    # def safeflatAlgo(x, y):
    #     to_x = -0.1 if (x-320) < 0 else 0.1
    #     to_y = -0.1 if (y-240) < 0 else 0.1
    #     to_z = 0
    #     return np.array([to_x,to_y,to_z])

    def moveTo(self, global_poses, calcIK, x, y):
        vec3 = User.flatAlgo(global_poses, x, y)
        # vec3 = User.safeflatAlgo(x, y)
        print(f"moveTo vector {vec3=}")
        print(joints := calcIK(vec3, None))
        self.pose = joints

    def setTargets(self, centers, ids):
        if len(centers) == 2:
            # self.target_x = centers[0][0] + centers[1][0]
            self.target_x, self.target_y = [(a+b)/2 for a,b in zip(centers[0], centers[1])]
        else:
            center = centers[0]
            id = ids[0]
            # self.target_x = center[0]
            self.target_x = center[0]-60 if id == 1 else center[0]+60
            self.target_y = center[1]
            # self.target_y = center[1]-45 if id == 1 else center[1]+45
        print(f"set targets {self.target_x=} {self.target_y=}")

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

        at_detector = Detector()
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(grey)

        # info output

        if tags:
            centers = []
            ids = []
            for tag in tags:
                if tag.tag_id:
                    print("this tag is furthest from the manipulator base")
                else:
                    print("this tag is nearest to the manipulator base")

                print(str(tag.center))
                print(str(tag.corners))
                start_x = int(tag.corners[0][0])
                start_y = int(tag.corners[0][1])
                end_x = int(tag.corners[2][0])
                end_y = int(tag.corners[2][1])
                cv2.rectangle(image, (start_x, start_y), (end_x, end_y), (255, 0, 255), 3)
                centers.append(tag.center)
                ids.append(tag.tag_id)

            self.setTargets(centers, ids)
            self.moveTo(global_poses, calcIK, self.target_x, self.target_y)
            self.locking = True
        elif not self.locking:
            print("hello")
            self.pose = calcIK(self.default, None)
            self.default = [x+(random()-random())*2 for x in [0.5, 0, 0]]
        elif self.target_x != None and self.target_x != None:
            print(f"continue to {self.target_x=} {self.target_y=}")
            self.moveTo(global_poses, calcIK, self.target_x, self.target_y)
            self.moving += 1
            if (self.moving > 5):
                self.locking = False

        # end info output

        cv2.imshow("View", image)
        cv2.waitKey(1)

        return self.pose

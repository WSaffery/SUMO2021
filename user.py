# From micah: My git was broken during this hackathon so I was drag and dropping files to share to my team.
# Chucked in some comments so people know I was here lol.
""" User code lives here """
import time
from typing import Dict
import math
from typing import Callable, Optional
import numpy as np
import cv2
from enum import Enum

height = 480
width = 640

# the specification says use width, but we discovered height worked? Questionable.
fmco=height/(2 * math.tan((100/2) * (math.pi / 180)))
matrix_coefficients = np.array([[fmco,0,width/2],[0,fmco,height/2],[0,0,1]])

# Beep boop beep boop
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
        self.vec = [0,0,0]
        self.quat = [0,0,0,0]
        self.targets = {} # We record targets
        self.targetLastUpdate = {} # and when they were last recorded for trashing
        self.state = RoboStates.Searching # State machine stuff
        self.grabTarget = []
        self.trueOffsets = {} # This isn't actually used but I attempted to make the offset more perfect using the rvects. 
        return

    def quaternionRotationMatrix(self, Q): # compressed jay's implementation. I just shortened this to get the line count down.
        return np.array([[2 * (Q[0] * Q[0] + Q[1] * Q[1]) - 1, 2 * (Q[1] * Q[2] - Q[0] * Q[3]), 2 * (Q[1] * Q[3] + Q[0] * Q[2])],
                        [2 * (Q[1] * Q[2] + Q[0] * Q[3]), 2 * (Q[0] * Q[0] + Q[2] * Q[2]) + 1, 2 * (Q[2] * Q[3] - Q[0] * Q[1])], # the +1 instead of a -1 here was discovered while randomly configuring variables. The current consensus between developers is that this was the affect of divine intervention.
                        [2 * (Q[1] * Q[3] - Q[0] * Q[2]), 2 * (Q[2] * Q[3] + Q[0] * Q[1]), 2 * (Q[0] * Q[0] + Q[3] * Q[3]) - 1]])

    # This took god long amounts of research. Multiple devs were up until 2am, reading research papers, implementations of opengl, etc.
    def inverseCameraProjection(self, camera_pos, camera_angle, pos):
        return -np.matmul(self.quaternionRotationMatrix(camera_angle), pos) + camera_pos

    # This is just to make it easy to reference the previous frame. (We don't heavily rely on this in the end)
    def setPose(self, calcIK, vec, quat):
        self.vec = vec
        self.quat = quat
        self.pose = calcIK(vec, quat)

    def run(self, image: list,  global_poses: Dict[str, np.ndarray], calcIK: Callable[[np.ndarray, Optional[np.ndarray]], Dict[str, float]]) -> Dict[str, float]:

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        tags = []
        if corners: # the magic with the projection. We wasted a lot of time using 0.02 since the example we found did it that way and we assumed it was irrelevant.
            for tag,id in zip(corners, ids):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(tag, 0.06, matrix_coefficients, distCoeffs=None)
                self.targets[id[0]] = self.inverseCameraProjection(global_poses["camera_end_joint"][0], global_poses["camera_end_joint"][1], tvec[0][0])
                self.targetLastUpdate[id[0]] = time.time()
                tags.append([tvec[0][0], id[0]])
                cv2.aruco.drawAxis(image, matrix_coefficients, None, rvec, tvec, 0.03)

        n_tags = len(self.targets)
        image = cv2.putText(image, self.state.name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0)) #debugging purposes
        if tags: # n_tags differs from len(tags) since it refers to the dictionary aka the "memory" rather than the currently visible tags
            if (n_tags==1):
                self.state = RoboStates.Located_1
            elif (n_tags==2):
                self.state = RoboStates.Located_2

        # This is the state machine. Me and Will (the one and only) had many disagreements about this.
        if self.state == RoboStates.Searching:
            if 0 in self.targets and ((time.time()-self.targetLastUpdate[0])>0.3):
                 del self.targets[0]
            if 1 in self.targets and ((time.time()-self.targetLastUpdate[1])>0.3):
                 del self.targets[1]
            # We all spent a ton of time implementing different ideas, then I put in a straight line and it beat everything else.
            # Someone suggested making this into a box so it technically covers the entire area but we decided this was good enough for now.
            on_line_pos = np.array([0.65, 0.15, 0.35]) + math.sin(time.time()) * np.array([0.3, 0.3, 0])
            # We worked with euler angles for most of the project, then converted to quaternions in the final release.
            self.setPose(calcIK, on_line_pos, [0, math.sqrt(1/2), math.sin(time.time()), math.sqrt(1/2)])

            # This might look a tinsy bit redundant since we worked on assuming we can see both tags, then made it support a single tag afterwards. 
        if self.state == RoboStates.Located_1:
            if 0 in self.targets:
                target_pos = self.targets[0]
                self.grabTarget = (target_pos[0]+0.15,target_pos[1],target_pos[2])
            if 1 in self.targets:
                target_pos = self.targets[1]
                self.grabTarget = (target_pos[0]-0.15,target_pos[1],target_pos[2])
            self.state = RoboStates.Grabbing
            self.enteredGrabbing = time.time()

        if self.state == RoboStates.Located_2:
            self.grabTarget = (self.targets[0]+self.targets[1])/2
            self.state = RoboStates.Grabbing
            self.enteredGrabbing = time.time()

        if self.state == RoboStates.Grabbing:
            self.setPose(calcIK, self.grabTarget, None)
            # self.setPose(calcIK, self.grabTarget+np.array([math.sin(time.time()*3)*0.01,math.sin(time.time()*3)*0.01,0]), None)
            if (time.time()-self.enteredGrabbing)>2: # Timer worked better than our 'smarter' memory stuff.
                self.state = RoboStates.Searching
                self.delaySearch = 1
                self.targets = {} # Reset targets so you don't instantly enter the same state again
                self.setPose(calcIK, np.array([0.5, 0, 0.5]), None) # Originally we over-engineered the quaternions, making it always face down, but found that if we left it to None, it'd use the best value anyway.
                time.sleep(0.5)

        cv2.imshow("View", image)
        cv2.waitKey(1)
        return self.pose

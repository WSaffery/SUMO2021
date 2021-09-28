""" User code lives here """

from typing import Dict
import math
from typing import Callable, Optional
import numpy as np

class User:
    def __init__(self) -> None:
        self.pose = {
            "bravo_axis_a": 0,
            "bravo_axis_b": 0,
            "bravo_axis_c": math.pi * 0.,
            "bravo_axis_d": math.pi,
            "bravo_axis_e": math.pi * 0.0,
            "bravo_axis_f": math.pi * 0.5,
            "bravo_axis_g": math.pi
        }
        self.inc = 0.1
        return
    
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
            A dictionary with the global camera and end effector positions. Keys are
            'camera_end_joint' and 'end_effector_joint', values are vec3.
        
        calcIK: function, (pos: np.ndarray, orient: np.ndarray = None) -> Dict[str, float]
            Function to calculate inverse kinematics. Provide a desired end effector
            position (vec3) and an orientation (quaternion) and it will return a pose
            dictionary of joint angles to approximate the pose.
        """

        # THIS IS AN EXAMPLE TO SHOW YOU HOW TO MOVE THE MANIPULATOR
        if self.pose["bravo_axis_f"] > math.pi:
            self.inc = -0.1
        if self.pose["bravo_axis_f"] < math.pi * 0.5:
            self.inc = 0.1
        self.pose["bravo_axis_f"] += self.inc

        # EXAMPLE USAGE OF INVERSE KINEMATICS SOLVER
        #   Inputs: vec3 position, quaternion orientation
        # self.pose = calcIK(np.array([0.8, 0, 0.4]), np.array([1, 0, 0, 0]))

        return self.pose
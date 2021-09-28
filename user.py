""" User code lives here """

from typing import Dict
import math
from typing import Callable, Optional
import numpy as np

class User:
    def __init__(self) -> None:
        self.pose = {
            "bravo_7_axis_a": 0, 
            "bravo_7_axis_b": 0, 
            "bravo_7_axis_c": math.pi * 0.,
            "bravo_7_axis_d": math.pi, 
            "bravo_7_axis_e": math.pi * 0.0, 
            "bravo_7_axis_f": math.pi * 0.5, 
            "bravo_7_axis_g": math.pi
        }
        self.inc = 0.1
        return
    
    def run(self, 
        image: list, 
        new_pose: Dict[str, float], 
        calcIK: Callable[[np.ndarray, Optional[np.ndarray]], Dict[str, float]],
    ) -> Dict[str, float]:
        # THIS IS AN EXAMPLE TO SHOW YOU HOW TO MOVE THE MANIPULATOR
        if self.pose["bravo_7_axis_f"] > math.pi:
            self.inc = -0.1
        if self.pose["bravo_7_axis_f"] < math.pi * 0.5:
            self.inc = 0.1
        self.pose["bravo_7_axis_f"] += self.inc

        return self.pose
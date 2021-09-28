import math
from typing import Callable, Dict, List, Tuple
import pybullet as p
import random

BASE_INDEX = -1
ORIGIN = [0, 0, 0]



class UUV:
    id: int
    def __init__(self, roundNum) -> None:
        init_pos = [0.5, 0, -0.05]
        maxDist = 0.15
        random_pos = [2.0*maxDist*(random.random()-0.5) + init_pos[i] for i in range(2)] \
            + [init_pos[2]]

        self.id: int = p.loadURDF("uuv/uuv.urdf",
            basePosition=random_pos,
            # baseOrientation=p.getQuaternionFromEuler([math.pi * 0.5, 0, 0])
        )

        self.round = roundNum # which competition round? 1, 2 or 3.

        self.timeScale = 0.016
        self.maxVel2 = 0.05
        self.phase2 = 2.0*random.random()*math.pi

        maxPrdScale = 0.1
        self.prdScales = [1.0 + maxPrdScale*2.0*(random.random() -0.5) for i in range(3)]
        self.phase3 = [
            2.0*random.random()*math.pi,
            2.0*random.random()*math.pi,
            math.pi
        ]
        self.maxVel3 = [0.03, 0.045, 0.02]
        
        # self.PERTURB_FNS: Dict[str, Callable] = { 
        #     "random": self.random_perturbation,
        #     "sine": self.sine_perturbation,
        # }
        # self.perturb_mode: str = "sine"
        # self.sine_x = random.random()
        # self.sine_y = random.random()
        # self.sine_z = random.random()
    
    def run(self, ticks):
        if self.round == 2:
            return p.resetBaseVelocity(
                self.id,
                linearVelocity=[0, self.maxVel2*math.sin(self.timeScale*ticks + self.phase2), 0]
            )

        if self.round == 3:
            return p.resetBaseVelocity(
                self.id,
                linearVelocity=[
                    self.maxVel3[i]*math.sin(self.prdScales[i]*self.timeScale*ticks + self.phase3[i])
                    for i in range(3)
                ]
            )

        return

    # def get_perturbation(self) -> List[float]:
    #     return self.PERTURB_FNS[self.perturb_mode]()
        
    # def random_perturbation(self) -> List[float]:
    #     perturb = [random.random() * 0.01 for _ in range(3)]
    #     return perturb

    # def sine_perturbation(self) -> List[float]:
    #     self.sine_x += 0.01*random.random()
    #     self.sine_y += 0.01*random.random()
    #     self.sine_z += 0.01*random.random()
    #     perturb = [k*math.sin(x) for x, k in [(self.sine_x, 0.3), (self.sine_y, 0.4), (self.sine_z, 0.1)]]
    #     return perturb

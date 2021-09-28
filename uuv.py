import math
from typing import Callable, Dict, List, Tuple
import pybullet as p
import random

BASE_INDEX = -1
ORIGIN = [0, 0, 0]



class UUV:
    id: int
    def __init__(self) -> None:
        self.id: int = p.loadURDF("uuv/uuv.urdf",
            basePosition=[0, 0, -0.5],
            # baseOrientation=p.getQuaternionFromEuler([math.pi * 0.5, 0, 0])
        )

        self.PERTURB_FNS: Dict[str, Callable] = { 
            "random": self.random_perturbation,
            "sine": self.sine_perturbation,
        }
        self.perturb_mode: str = "sine"

        self.sine_x = random.random()
        self.sine_y = random.random()
        self.sine_z = random.random()
    
    def run(self):
        # p.applyExternalForce(self.id,
        #                     linkIndex=BASE_INDEX,
        #                     forceObj=self.get_perturbation(),
        #                     posObj=ORIGIN,
        #                     flags=p.LINK_FRAME)
        # p.resetBaseVelocity(self.id,
                            # linearVelocity=self.get_perturbation()
                            # )
        return

    def get_perturbation(self) -> List[float]:
        return self.PERTURB_FNS[self.perturb_mode]()
        
    def random_perturbation(self) -> List[float]:
        perturb = [random.random() * 0.01 for _ in range(3)]
        return perturb

    def sine_perturbation(self) -> List[float]:
        self.sine_x += 0.01*random.random()
        self.sine_y += 0.01*random.random()
        self.sine_z += 0.01*random.random()
        perturb = [k*math.sin(x) for x, k in [(self.sine_x, 0.3), (self.sine_y, 0.4), (self.sine_z, 0.1)]]
        return perturb

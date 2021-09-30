import pybullet as p
import math

visualProps = {}

def updateProp(name, vec3, use3D):
    if use3D:
        colours = ["R", "G", "B"]
        colour = colours[name]
        if name in visualProps:
            p.resetBasePositionAndOrientation(visualProps[name], vec3, p.getQuaternionFromEuler([0,math.pi, 0]))
        else:
            visualProps[name]: int = p.loadURDF(f"./sphere{colour}.urdf", basePosition = vec3)

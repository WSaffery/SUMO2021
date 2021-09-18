import pybullet as p
import time 


def run():
    p.connect(p.GUI)
    p.loadURDF("Blueprint_Lab_Software/ROS/bpl_bravo_description/urdf/bravo_7.urdf")
    time.sleep(100)


if __name__ == "__main__":
    run()
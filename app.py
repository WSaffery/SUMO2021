import pybullet as p
import pybullet_data
import time


class App:
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

        bravo_id = p.loadURDF("bpl_bravo_description/urdf/bravo_7.urdf")
        uuv_id = p.loadURDF("uuv/uuv.urdf")
        return

    def run(self):
        time.sleep(30)
        return
    
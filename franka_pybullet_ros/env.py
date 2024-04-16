import os
import pkgutil
import pybullet as p
import pybullet_data

from panda_robot import FrankaPanda
import pybullet_utils.bullet_client as bc


class FrankaPandaEnv:
    def __init__(self, connection_mode=p.GUI, frequency=1000., controller='position',
                 include_gripper=True, simple_model=False,
                 object_from_sdf=False, object_from_list=False):

        self.bc = bc.BulletClient(connection_mode)
        egl = pkgutil.get_loader('eglRenderer')
        self.bc.loadPlugin(egl.get_filename(), "eglRendererPlugin")
        self.bc.configureDebugVisualizer(rgbBackground=[33. / 255., 90. / 255., 127. / 255.])
        self.bc.configureDebugVisualizer(self.bc.COV_ENABLE_GUI, 0)
        self.frequency = frequency
        self.bc.setTimeStep(1 / frequency)
        self.bc.setGravity(0, 0, -9.81)

        self.plane_id = None
        self.table_id = None
        self.object_id = []
        self.object_list = ["YcbBanana", "YcbPear", "YcbHammer", "YcbScissors", "YcbStrawberry", "YcbChipsCan",
                            "YcbCrackerBox", "YcbFoamBrick", "YcbGelatinBox", "YcbMasterChefCan",
                            "YcbMediumClamp",
                            "YcbMustardBottle", "YcbPottedMeatCan", "YcbPowerDrill", "YcbTennisBall",
                            "YcbTomatoSoupCan"]

        if object_from_list:
            self.add_ycb_objects_from_list(self.object_list)
        elif object_from_sdf:
            self.add_ycb_objects_from_sdf('./grasp_sdf_env/layout_1.sdf')

        self.controller = controller
        self.include_gripper = include_gripper
        self.panda_robot = FrankaPanda(self.bc, include_gripper=include_gripper, simple_model=simple_model)

    def simulate_step(self):
        self.bc.stepSimulation()

    def simulation_loop(self):
        import time
        while True:
            start = time.time()
            self.simulate_step()
            print(1 / (time.time() - start))
        self.bc.disconnect()
        print("Simulation end")

    def add_urdf_object(self, filename):
        flags = self.bc.URDF_USE_INERTIA_FROM_FILE
        self.object_id.append(self.bc.loadURDF(filename, [0.5, 0.0, 0.4], flags=flags))

    def add_ycb_objects_from_list(self, object_list):
        self.bc.configureDebugVisualizer(self.bc.COV_ENABLE_RENDERING, 0)

        self.plane_id = self.bc.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"),
                                         basePosition=[0, 0, -0.65], useFixedBase=True)
        self.table_id = self.bc.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),
                                         basePosition=[0.5, 0, -0.65], useFixedBase=True)

        print("Loading objects from a list")
        for obj in object_list:
            self.add_urdf_object("./ycb_objects/" + obj + "/model.urdf")
            for i in range(200):
                self.bc.stepSimulation()
        self.bc.configureDebugVisualizer(self.bc.COV_ENABLE_RENDERING, 1)

    def add_ycb_objects_from_sdf(self, object_sdf):
        self.bc.configureDebugVisualizer(self.bc.COV_ENABLE_RENDERING, 0)

        self.plane_id = self.bc.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"),
                                         basePosition=[0, 0, -0.65], useFixedBase=True)
        self.table_id = self.bc.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),
                                         basePosition=[0.5, 0, -0.65], useFixedBase=True)

        print("Loading objects from a SDF")
        object_id_temp = self.bc.loadSDF(object_sdf)
        for id in object_id_temp:
            pos, orn = self.bc.getBasePositionAndOrientation(id)
            pos = (pos[0] + 0.9, pos[1] - 0.15, pos[2])
            self.bc.resetBasePositionAndOrientation(id, pos, orn)
        self.object_id += object_id_temp

        self.bc.configureDebugVisualizer(self.bc.COV_ENABLE_RENDERING, 1)

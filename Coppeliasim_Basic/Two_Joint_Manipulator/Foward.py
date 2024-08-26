from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np



class TwoJointManipulator():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.t = 0

    def init_coppelia(self):
        self.joint1 = self.sim.getObject("/Joint1")
        self.joint2 = self.sim.getObject("/Joint2")
        self.endeff = self.sim.getObject("/EndEff")
        self.endeff_tracing = self.sim.addDrawingObject(self.sim.drawing_linestrip, 5, 0, -1, 100000, [0, 1, 0])

    def control(self):
        self.theta1 = self.sim.getJointPosition(self.joint1)
        self.theta2 = self.sim.getJointPosition(self.joint2)
        print("theta1 =", self.theta1, "theta2 =", self.theta2)

        self.t = self.sim.getSimulationTime()
        self.theta1 = (np.pi * self.t) / 6
        self.theta2 = (np.pi * self.t) / 3
        self.sim.setJointTargetPosition(self.joint1, self.theta1)
        self.sim.setJointTargetPosition(self.joint2, self.theta2)

    def sensing(self):
        self.endeff_pos = self.sim.getObjectPosition(self.endeff, self.sim.handle_world)
        self.sim.addDrawingObjectItem(self.endeff_tracing, self.endeff_pos)

    def run(self):
        self.sim.setStepping(True)
        self.sim.startSimulation()
        while (True):
            self.t = self.sim.getSimulationTime()
            if self.t > 20:
                break
            self.control()
            self.sensing()
            self.sim.step()
        self.sim.stopSimulation()

if __name__ == "__main__":
    env = TwoJointManipulator()
    env.init_coppelia()
    env.run()
    

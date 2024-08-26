from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np

T_end = 10
r = 0.5

class TwoJointManipulator_Inverse():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.t = 0

    def curve(self, t):
        self.f = 2 * np.pi / T_end
        self.x_ref = self.x_center + r * np.cos(self.f * t)
        self.y_ref = self.y_center + r * np.sin(self.f * t)
        return self.x_ref, self.y_ref  

    def init_coppelia(self):
        self.joint1 = self.sim.getObject("/Joint1")
        self.joint2 = self.sim.getObject("/Joint2")
        self.endeff = self.sim.getObject("/EndEff")
        self.endeff_tracing = self.sim.addDrawingObject(self.sim.drawing_linestrip, 5, 0, -1, 100000, [0, 1, 0])
        self.endeff_pos = self.sim.getObjectPosition(self.endeff, -1)
        self.x = self.endeff_pos[0]
        self.y = self.endeff_pos[1]
        self.x_center = self.x - r
        self.y_center = self.y

    def control(self):
        self.theta1 = self.sim.getJointPosition(self.joint1)
        self.theta2 = self.sim.getJointPosition(self.joint2)
        print("theta1 =", self.theta1, "theta2 =", self.theta2)

        self.l = 1
        self.J = np.array((
            [self.l * np.cos(self.theta1) + self.l * np.cos(self.theta1 + self.theta2), self.l * np.cos(self.theta1 + self.theta2)],
            [self.l * np.sin(self.theta1) + self.l * np.sin(self.theta1 + self.theta2), self.l * np.sin(self.theta1 + self.theta2)]
        ))
        
        try:
            self.J_inv = np.linalg.inv(self.J)
        except np.linalg.LinAlgError:
            print("J is singular.")
            return

        self.t = self.sim.getSimulationTime()
        self.x_ref, self.y_ref = self.curve(self.t)
        self.X_ref = [self.x_ref, self.y_ref]
        self.endeff_pos = self.sim.getObjectPosition(self.endeff, -1)
        self.x = self.endeff_pos[0]
        self.y = self.endeff_pos[1]

        self.dr = [self.x_ref - self.x, self.y_ref - self.y]
        self.dq = np.dot(self.J_inv, self.dr)

        self.theta1 += self.dq[0]
        self.theta2 += self.dq[1]
        self.sim.setJointTargetPosition(self.joint1, self.theta1)
        self.sim.setJointTargetPosition(self.joint2, self.theta2)

        if self.t >= T_end:
            self.sim.stopSimulation()  

    def sensing(self):
        self.endeff_pos = self.sim.getObjectPosition(self.endeff, self.sim.handle_world)
        self.sim.addDrawingObjectItem(self.endeff_tracing, self.endeff_pos)

    def run(self):
        self.sim.setStepping(True)
        self.sim.startSimulation()
        while self.t < T_end:  
            self.control()
            self.sensing()
            self.sim.step()
            self.t = self.sim.getSimulationTime()  
        self.sim.stopSimulation()

if __name__ == "__main__":
    env = TwoJointManipulator_Inverse()
    env.init_coppelia()
    env.run()

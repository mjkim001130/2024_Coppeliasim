from coppeliasim_zmqremoteapi_client import RemoteAPIClient

FSM_velocity = 0
FSM_position = 1
FSM_damping = 2

class Pendulumn:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

    def init_coppelia(self):
        self.joint = self.sim.getObject("/Joint")
        self.t_position = 0
        self.t_damping = 0

        self.FSM = FSM_velocity

    def control_pendulum(self):
        
        self.t = self.sim.getSimulationTime()
        self.theta = self.sim.getJointPosition(self.joint)
        self.theta_dot = self.sim.getJointVelocity(self.joint)
            
        if self.FSM == FSM_velocity and self.theta > -0.5 and self.theta < 0.5:
            self.t_position = self.t
            self.FSM = FSM_position
            print("Transition to FSM_position")
               
        if self.FSM == FSM_position and self.t - self.t_position > 3:
            self.t_damping = self.t
            self.FSM = FSM_damping
            print("Transition to FSM_damping")
        
        if self.FSM == FSM_damping and self.theta_dot > -0.1 and self.theta_dot < 0.1 and self.t - self.t_damping > 4:
            self.FSM = FSM_velocity
            self.sim.setJointTargetForce(self.joint, 10000)
            print("Transition to FSM_velocity")

        if self.FSM == FSM_velocity:
            self.sim.setObjectInt32Param(
                self.joint,
                self.sim.jointintparam_dynctrlmode,
                self.sim.jointdynctrl_velocity
            )
            self.sim.setJointTargetVelocity(self.joint, 0.5)

        if self.FSM == FSM_position:
            self.sim.setObjectInt32Param(
                self.joint,
                self.sim.jointintparam_dynctrlmode,
                self.sim.jointdynctrl_position
            )
            self.sim.setJointTargetPosition(self.joint, 0.1)
        
        if self.FSM == FSM_damping:
            Force = -0.5 * self.theta_dot
            self.sim.setObjectInt32Param(
                self.joint,
                self.sim.jointintparam_dynctrlmode,
                self.sim.jointdynctrl_force
            )
            self.sim.setJointTargetPosition(self.joint, Force)

    def run_coppelia(self, sec):

        self.sim.setStepping(True)
        self.sim.startSimulation()

        while (t := self.sim.getSimulationTime()) < sec:
            self.control_pendulum()
            self.sim.step()
        self.sim.stopSimulation()

if __name__ == '__main__':
    pendulumn = Pendulumn()
    pendulumn.init_coppelia()
    pendulumn.run_coppelia(100)
        


        



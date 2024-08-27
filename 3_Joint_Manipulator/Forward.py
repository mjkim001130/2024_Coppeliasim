from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np

def rot_mat(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

def homogeneous_matrix(theta, trans):
    H = np.identity(3)
    H[:2, :2] = rot_mat(theta)
    H[:2, 2] = np.array(trans)
    return H

class ForwardKinematics():
    def __init__(self):
        self.l = 1

    def compute_transform(self, thetas):
        transforms = []
        for theta in thetas:
            trans = [self.l, 0]
            transform = homogeneous_matrix(theta, trans)
            transforms.append(transform)
        return transforms
    
    def compute_endeffector(self, thetas):
        transforms = self.compute_transform(thetas)
        
        T = np.identity(3)
        position = [T[:2, 2]]

        for transform in transforms:
            T = np.dot(T, transform)
            position.append(T[:2, 2])

        return position


class ThreeJointManipulator():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        joint_names = ["/Joint1", "/Joint2", "/Joint3"]
        self.joint_handles = [self.sim.getObject(joint_name) for joint_name in joint_names]
        self.fk = ForwardKinematics()

    def control(self, thetas):
        for angle, theta in zip(self.joint_handles, thetas):
            self.sim.setJointPosition(angle, theta)

    def get_position(self, thetas):
        positions = self.fk.compute_endeffector(thetas)
        return positions

    def run(self):
        self.sim.setStepping(True)
        self.sim.startSimulation()

        try:
            while True:
                self.theta1 = float(input("Theta1 : ")) * np.pi / 180
                self.theta2 = float(input("Theta2 : ")) * np.pi / 180
                self.theta3 = float(input("Theta3 : ")) * np.pi / 180

                self.thetas = [self.theta1, self.theta2, self.theta3]

                self.control(self.thetas)

                positions = self.get_position(self.thetas)

                for i, pos in enumerate(positions):
                    if i == len(positions) - 1:
                        print(f"End-effector position: {pos}")
                    else:
                        print(f"Link{i + 1} position: {pos}")

                if input("Continue? (y/n): ").lower() != 'y':
                    break
        finally:
            self.sim.stopSimulation()
            print("Simulation stopped.")


if __name__ == "__main__":
    env = ThreeJointManipulator()
    env.run()




        
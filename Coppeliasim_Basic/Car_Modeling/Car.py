import math
import numpy as np

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class Car():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

    def init_coppelia(self):
        self.joint_right = self.sim.getObject('/Joint_right')
        self.joint_left = self.sim.getObject('/Joint_left')

    def euler_integration(tspan, z0, u):
        v, omega = u
        h = tspan[1] - tspan[0]
        x0, y0, theta = z0

        xdot_c = v * math.cos(theta)
        ydot_c = v * math.sin(theta)
        thetadot_c = omega

        x1 = x0 + xdot_c * h
        y1 = y0 + ydot_c * h
        theta1 = theta + thetadot_c * h

        return [x1, y1, theta1]
    
    def control_car(self):

        # initial condition [x0, y0, theta0]
        z0 = [0, 0, -math.pi/2]

        t1 = np.arange(0, 1, 0.1)
        t2 = np.arange(1, 2, 0.1)
        t3 = np.arange(2, 3, 0.1)

        for i in range(0 ,t1):
            z1 = self.euler_integration



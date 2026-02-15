import numpy as np
import matplotlib.pyplot as plt
from cbf_toolbox.geometry import Point, Sphere, Ellipsoid, HalfPlane
from cbf_toolbox.dynamics import Dynamics, SingleIntegrator2d, SingleIntegrator
from cbf_toolbox.vertex import Agent, Obstacle, Goal
from cbf_toolbox.safety import Simulation

def f_func(x):
    f0 = f1 = M = 1
    f2 = 0.2
    v_l = 2
    v = x[0]
    return np.array([-(1.0/M) * (f0 + f1*v + f2*v*v), (v_l - v)])

def g_func(x):
    M = 1
    return np.array([[1.0/M], [0]])

def const_vel_f(x):
    v = x[0]
    return np.array([0, v])

def const_vel_g(x):
    return np.zeros((2,2))

def main():
    v_l = 0.5 # leading vehicle velocity 
    v_d = 1 # desired velocity 
    tau = 1
    dyn = Dynamics(2, 1, f_func, g_func)

    # Create an agent with some dynamics 
    a_s0 = np.array([0.0, tau*v_l]) # 0 initial velocity, initial distance
    lagging_vehicle = Agent(state=a_s0, shape=Point(), dynamics=dyn) 
    
    # Create a goal 
    # leading vehicle moves forward at constant velocity v_l
    const_vel = Dynamics(2, 2, const_vel_f, const_vel_g)
    g1 = Goal(np.array([v_d, tau*v_l]), dynamics=const_vel, p=1, gamma=3, H=np.array([[1, 0], [0, 1]]))
    g1.color = 'blue'

    # Create an obstacle --> CBF 
    leading_vehicle = Obstacle(state=np.array([v_l, tau*v_l]), shape=Point(), dynamics=const_vel, k=5, p=5)

    
    # Now we add everything to a simulation object
    s = Simulation()
    s.add_agent(agent=lagging_vehicle, control=g1, upper_bounds=100, lower_bounds=-100)
    s.add_obstacle(obst=leading_vehicle)
    s.simulate(num_steps=100, dt = 0.1)

    s.plot_functions()

if __name__ == '__main__':
    main()
import numpy as np
import matplotlib.pyplot as plt
from cbf_toolbox.geometry import Point, Sphere, Ellipsoid, HalfPlane
from cbf_toolbox.dynamics import Dynamics, SingleIntegrator2d, SingleIntegrator
from cbf_toolbox.vertex import Agent, Obstacle, Goal
from cbf_toolbox.safety import Simulation



def main():
    # Dynamics  
    f0 = f1 = M = 1
    f2 = 0.2
    f = lambda v : -(1.0/M) * (f0 + f1*v + f2*v*v)
    g = lambda v : np.array([1.0/M])
    dyn = Dynamics(1, 1, f, g)

    # Create an agent --> CLF 
    a_s0 = np.array([0.0])
    agent = Agent(state=a_s0, shape=Point(1), dynamics=dyn)
    
    # Create a goal 
    g1 = Goal(np.array([5.0]), Point(1), dynamics=SingleIntegrator(1), gamma=0.8)
    g1.color = 'blue'
    
    # Now we add everything to a simulation object
    s = Simulation()
    s.add_agent(agent=agent, control=g1, upper_bounds=100, lower_bounds=-100)

    # When everything is added, we can call the simulate function
    # Before running the simulation, the function will loop over all the agents and obstacles and create
    # the proper Edges to connect the Vertex objects (the CBF and CLF objects)
    s.simulate(num_steps=100, dt = 0.1)

    s.plot_functions()

if __name__ == '__main__':
    main()
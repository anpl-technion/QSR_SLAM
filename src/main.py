import numpy as np


if __name__ == "__main__":
    pass
    # generate random map
    rmin = 0.01
    xlims = [-7, 7]
    ylims = [-5, 5]
    map = SimulationMap(xlims, ylims, rmin)
    map.add_landmarks(10)

    # generate random trajectory
    trajectory = SimulationTrajectory(xlims, ylims, rmin)
    trajectory.add_steps(10)

    # generate measurements
    trajectory = SimulationTrajectory(xlims, ylims, rmin)
    trajectory.add_steps(10)


    # plot scenario

    # solve QSR inference

    # analyse results
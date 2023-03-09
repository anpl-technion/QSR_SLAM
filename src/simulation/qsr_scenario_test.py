import matplotlib.pyplot as plt
import time
import numpy as np
import simulation


def manual_scenario_test():
    """
    test add / get/ remove landmarks
    """

    # configure scenario
    motion_model_noise_std = 0.1  # [deg]
    azimuth_measurement_noise_std = 0.1  # [deg]
    rmin = 0.01
    xlims = [-7, 7]
    ylims = [-5, 5]
    scen = simulation.qsr_scenario.Scenario(rmin, xlims, ylims, azimuth_measurement_noise_std, motion_model_noise_std)

    # add landmarks
    landmark_ids = range(0, 10)
    landmark_positions = [[-6, -4],
                          [-6, 1],
                          [-6, 4],
                          [-3, -4],
                          [-3, 1],
                          [-3, 4],
                          [3, -4],
                          [3, 1],
                          [3, 4],
                          [6, -4],
                          [6, 1],
                          [6, 4]]
    scen.add_landmarks(len(landmark_ids), landmark_ids=landmark_ids, landmark_positions=landmark_positions)

    # set trajectory
    view_ids = range(0, 12)
    view_poses = [[-4, -4, 135],
                  [-2, -2, 90],
                  [-2, 0, 90],
                  [-2, 2, 180],
                  [0, 2, 270],
                  [0, 0, 0]]
    scen.add_view(len(view_ids), view_ids=view_ids, view_poses=view_poses)

    # set view -> landmark observations
    observations_ids = [[0, 0],
                       [0, 1],
                       [0, 3],
                       [1, 1],
                       [1, 3],
                       [1, 4],
                       [2, 3],
                       [2, 4],
                       [2, 5],
                       [3, 4],
                       [3, 5],
                       [3, 6],
                       [4, 3],
                       [4, 4],
                       [4, 5],
                       [5, 4],
                       [5, 5],
                       [5, 7]]
    observation_data = range(0, len(observations_ids))
    scen.add_view_to_landmark_observations(view_ids=[x[0] for x in observations_ids],
                                           landmark_ids=[x[1] for x in observations_ids], data=observation_data)

    # set view -> view motion model
    motion_model_ids = [[x, x+1] for x in view_ids[0:-2]]
    observation_data = range(0, len(motion_model_ids))
    scen.add_view_to_view_observations(view_ids1=[x[0] for x in motion_model_ids],
                                       view_ids2=[x[0] for x in motion_model_ids], data=observation_data)

    # plot scenario
    scen.plot_map_lims(color=[])
    scen.plot_landmarks(color=[])
    scen.plot_views(color=[])
    scen.plot_view_to_landmark_observations(color=[])
    scen.plot_view_to_view_observations(color=[])

    return


def plot_map_test(ax):
    """
    test add / get/ remove landmarks
    """
    xlims = [-7, 7]
    ylims = [-5, 5]
    plt.plot([xlims[0], xlims[0], xlims[1], xlims[1], xlims[0]], [ylims[0], ylims[1], ylims[1], ylims[0], ylims[0]],
             axes=ax)

    p1 = [[1, 2], [3, -2], [4, 1], [-3, -3], [2, 0]]
    plt.plot([x[0] for x in p1], [x[1] for x in p1], 'ob', axes=ax)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('scenario')  # subplot 211 title
    plt.show(block=False)


def plot_trajectory_test(ax, point):
    """
    test add / get/ remove landmarks
    """
    p = plt.plot(point[0], point[1], 'or', axes=ax)

    return p


if __name__ == "__main__":

    fig, ax = plt.subplots()
    plot_map_test(ax)

    p2 = [[0, -4], [0, -2], [-2, 0], [-4, 2], [0, 3]]
    plt2 = []
    for i in range(0, len(p2)):
        for pi in plt2:
            pi.remove()
        plt2 = plot_trajectory_test(ax, p2[i])
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(1)

    manual_scenario_test()

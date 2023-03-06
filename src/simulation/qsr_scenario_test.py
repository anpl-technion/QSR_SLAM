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


if __name__ == "__main__":
    import plotly.express as px

    for i in range(0,5):
        fig = px.line(x=["a", "b", "c"], y=[1, 3, 2], title="sample figure")
        print(fig)
    # fig.add_trace(px.scatter(x=[0, 1, 2, 3, 4], y=[0, 1, 4, 9, 16]))
    # fig.add_scatter(x=[0, 1, 2, 3, 4], y=[0, 1, 4, 9, 16])
        fig.show()
    manual_scenario_test()

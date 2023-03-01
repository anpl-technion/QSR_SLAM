import numpy as np
import simulation

if __name__ == "__main__":

    # set noise levels
    motion_model_noise_std = 0.1  # [deg]
    azimuth_measurement_noise_std = 0.1  # [deg]

    # set map
    rmin = 0.01
    xlims = [-7, 7]
    ylims = [-5, 5]

    # generate scenario
    scen = simulation.qsr_scenario.QsrScenario(rmin, xlims, ylims, azimuth_measurement_noise_std, motion_model_noise_std)
    scen.add_landmarks(10)

    view_ids = range(0, 12)
    scen.add_pose(12, view_ids)

    scen.add_landmark_to_view_observations(10)

    view_pairs = np.array([view_ids[:-1], view_ids[1:]])
    scen.get_view_to_view_observations(view_pairs)

    # plot scenario



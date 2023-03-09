import numpy as np
import common
import matplotlib.pyplot as plt


class Scenario:
    """
    2D map with landmarks
    """
    def __init__(self, rmin, xlim, ylim, azimuth_measurement_noise_std, motion_model_noise_std):
        self.map_limits_x = xlim  # map limits [xmin,xmax]
        self.map_limits_y = ylim  # map limits [ymin,ymax]
        self.min_range_between_objects = rmin  # minimum distance between objects(landmarks and cameras)

        self.azimuth_measurement_noise_std = azimuth_measurement_noise_std * np.pi/180  # azimuth measurement noise std
        self.motion_model_noise_std = motion_model_noise_std * np.pi/180  # motion model noise std

        max_num_view_to_landmark = 1000
        view_to_landmark_data_size = 1  # [azimuth]
        max_num_view_to_view = 100
        view_to_view_data_size = 1  # [heading]
        self.landmark_view_set = common.landmark_view_set.LandmarkViewSet(max_num_view_to_landmark,
                                                                     view_to_landmark_data_size,
                                                                     max_num_view_to_view, view_to_view_data_size)

    def add_landmarks(self, num_landmarks, landmark_ids=None, landmark_positions=None):
        """
        manually add landmarks to map:
        num_landmarks - number of landmarks to add
        landmark_ids - landmark ids [nx1]
                       if None landmark ids in ascending order
        landmark_positions - landmark positions [nx2]
                       if None, randomly sampled within map_lims while keeping rmin
        """

        if (num_landmarks is None) or num_landmarks < 1:
            Exception('invalid input!')

        if landmark_ids is None:
            max_landmark_id = self.landmark_view_set.get_max_landmark_id
            landmark_ids = range(max_landmark_id, max_landmark_id + num_landmarks)

        if landmark_positions is None:
            landmark_positions = self._generate_random_positions(num_landmarks)

        self.landmark_view_set.add_landmark(num_landmarks, landmark_ids=landmark_ids, landmark_positions=landmark_positions)

        return

    def add_views(self, num_views, view_ids=None, view_poses=None):
        """
        add camera view to trajectory:
        num_views - number of landmarks to add
        view_ids - view ids [nx1]
                       if None, view ids in ascending order
        view_poses - view 2D poses [nx3] - [x,y,azimuth]
                       if None, randomly sampled within map_lims while keeping rmin
        """

        if (num_views is None) or num_views < 1:
            Exception('invalid input!')

        if view_ids is None:
            max_view_id = self.landmark_view_set.get_max_view_id
            view_ids = range(max_view_id, max_view_id + num_views)

        if view_poses is None:
            view_poses = self._generate_random_poses(num_views)

        self.landmark_view_set.add_view(num_views, view_ids=view_ids, view_poses=view_poses)

        return

    def add_view_to_landmark_observations(self, view_id, landmark_id):
        """
        add view to landmark observation:
        view_id - view ids [nx1]
        landmark_id - landmark ids [nx1]
        """
        self.landmark_view_set.get_view_to_landmark_observations(view_ids=view_id, landmark_ids=landmark_id)

        return True

    def add_view_to_view_observations(self, view_id1, view_id2, measurement_std):
        """
        add view to landmark observation:
        view_id - view ids [nx1]
        landmark_id - landmark ids [nx1]
        """
        self.landmark_view_set.get_view_to_landmark_observations(view_ids1=view_id1, view_ids2=view_id2)

        return True

    def _generate_random_positions(self, num_landmarks):
        """
        randomize landmark position keeping:
        - self.xlim
        - self.ylim
        - self.rmin
        """

        positions = np.zeros((num_landmarks, 2))
        i = 0
        while i < num_landmarks:
            # random positions
            x = np.random.randint(self.xlim[0], high=self.xlim[1], size=(1, positions))
            y = np.random.randint(self.ylim[0], high=self.ylim[1], size=(1, positions))

            d1 = np.sqrt(np.power((x, y) - self.landmark_positions, 2))
            # d2 = np.sqrt(np.power((x, y) - self.view_positions, 2))
            d2 = np.inf
            # TODO: add poses

            if min(d1) > self.rmin and min(d2) > self.rmin:
                positions[i, :] = (x, y)
                i = i + 1

        return positions

    def _generate_random_pose(self, num_poses):
        """
        randomize poses keeping:
        - self.xlim
        - self.ylim
        - self.rmin
        """
        poses = np.zeros((num_poses, 3))
        positions = self._generate_random_positions(num_poses)
        az = np.random.rand((num_poses, 1)) * np.pi * 2
        poses[:, 1:] = positions
        az[:, 2] = az

        return poses

    def print(self):
        """
        print map data
        """
        pass

    def plot_map_lims(self, axes, color=None):
        """
        plot map limits
        """
        # plot map limits
        p1 = plt.plot([self.map_limits_x[0], self.map_limits_x[0], self.map_limits_x[1], self.map_limits_x[1], self.map_limits_x[0]],
                 [self.map_limits_y[0], self.map_limits_y[1], self.map_limits_y[1], self.map_limits_y[0], self.map_limits_y[0]],
                 axes=axes)

        # plot landmarks
        [is_valid, landmark_positions] = self.landmark_view_set.get_lanmark_positions()
        p = plt.plot(landmark_positions[:, 0], landmark_positions[:, 1], 'ob', axes=axes)

        return p

    def plot_landmarks(self, axes, landmark_ids=None):
        """
        plot landmarks
        axes - axes to plot in
        landmark_ids - plot specific landmark ids
                       None - plot all landmarks
        """
        [is_valid, landmark_positions] = self.landmark_view_set.get_lanmark_positions(landmark_ids)
        p = plt.plot(landmark_positions[:, 0], landmark_positions[:, 1], 'ob', axes=axes)
        return p

    def plot_views(self, axes, view_ids=None, az_los_size=None):
        """
        plot views
        axes - axes to plot in
        view_ids - plot specific view ids
                   None - plot all views
        """
        [is_valid, view_poses] = self.landmark_view_set.get_view_poses(view_ids)
        # plot position
        p1 = plt.plot(view_poses[:, 0], view_poses[:, 1], 'or', axes=axes)

        # plot direction
        # TODO: plot direction
        dx = np.abs(self.map_limits_x[1] - self.map_limits_x[0])
        dy = np.abs(self.map_limits_y[1] - self.map_limits_y[0])
        if az_los_size is None:
            sz = min(dx, dy) / 5
        else:
            sz = az_los_size
        x = view_poses[0, :]
        y = view_poses[1, :]
        az = view_poses[2, :]
        xx = x + sz * np.cos(az)
        yy = y + sz * np.sin(az)
        p2 = plt.plot([x, xx], [y, yy], '-r', axes=axes)  # TODO: is this correct?

        # plot trajectory
        p3 = plt.plot(x, y, '-r', axes=axes)  # TODO: is this correct?

        p = p1.extend(p2).extend(p3)

        return p

    def plot_view_to_landmark_observations(self, axes, view_id=None, landmark_id=None, color=None):
        """
        plot landmarks
        """

        [is_valid, view_poses] = self.landmark_view_set.get_view_to_landmark_observations(view_id, landmark_id)

        x = view_poses[:, 0]
        y = view_poses[:, 1]

        xx = view_poses[:, 0]
        yy = view_poses[:, 1]

        p = plt.plot([x, xx], [y, yy], '-r', axes=axes)  # TODO: is this correct?

        return p


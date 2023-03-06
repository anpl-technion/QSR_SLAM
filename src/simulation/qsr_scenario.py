import numpy as np
import common


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
        self.number_of_landmarks = 0
        self.number_of_views = 0

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
            landmark_ids = np.array(range(self.number_of_landmarks, self.number_of_landmarks+num_landmarks))

        if landmark_positions is None:
            landmark_positions = self._generate_random_positions(num_landmarks)
        else:

            if landmark_positions.shape != ():
                max_id = -1
            else:
                max_id = max(self.landmark_ids)

        self.landmark_view_set.add_landmark(num_landmarks, landmark_ids=landmark_ids, landmark_positions=landmark_positions)
        self.number_of_landmarks = self.number_of_landmarks + num_landmarks

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

        if num_views is None:
            if self.view_ids.shape[0] == 0:
                max_id = -1
            else:
                max_id = max(self.view_ids)
            view_ids = np.array(range(max_id + 1, max_id + 1 + num_views))
        self.view_ids = np.hstack((self.view_ids, view_ids))

        if num_views is None:
            view_poses = self._generate_random_positions(num_views)
        self.view_poses = np.vstack((np.array(self.view_poses), np.array(view_poses)))

        return

    def add_view_to_landmark_observations(self, view_id, landmark_id, measurement_std):
        """
        add view to landmark observation:
        view_id - view ids [nx1]
        landmark_id - landmark ids [nx1]
        """
        n = view_id.shape[0]
        if (view_id.shape is not (n, 1)) or (landmark_id.shape is not (n, 1)):
            Exception('invalid input!')

        is_valid = np.array((n, 3), dtype=np.bool)
        for i, vid in enumerate(view_id):
            if vid in self.view_ids and landmark_id[i] in self.landmark_ids:
                [view_pose, is_valid_i] = self.get_view(vid)
                [landmark_position, is_valid_i] = self.get_landmark(landmark_id[i])

                az_gt = np.arctan2(landmark_position[1] - view_pose[1], landmark_position[0] - view_pose[0]) - view_pose[2]
                az = az_gt + (np.random.randn(1, 1) * measurement_std)
                v2t = np.array([vid, landmark_id[i], az_gt, az])

                self.view_to_landmark_observations = np.vstack(self.view_to_landmark_observations, v2t)
                is_valid[i] = True
            else:
                is_valid[i] = False

        return is_valid

    def add_view_to_view_observations(self, view_id1, view_id2, measurement_std):
        """
        add view to landmark observation:
        view_id - view ids [nx1]
        landmark_id - landmark ids [nx1]
        """
        n = view_id.shape[0]
        if (view_id.shape is not (n, 1)) or (landmark_id.shape is not (n, 1)):
            Exception('invalid input!')

        is_valid = np.array((n, 3), dtype=np.bool)
        for i, vid in enumerate(view_id):
            if vid in self.view_ids and landmark_id[i] in self.landmark_ids:
                [view_pose, is_valid_i] = self.get_view(vid)
                [landmark_position, is_valid_i] = self.get_landmark(landmark_id[i])

                az_gt = np.arctan2(landmark_position[1] - view_pose[1], landmark_position[0] - view_pose[0]) - view_pose[2]
                az = az_gt + (np.random.randn(1, 1) * measurement_std)
                v2t = np.array([vid, landmark_id[i], az_gt, az])

                self.view_to_landmark_observations = np.vstack(self.view_to_landmark_observations, v2t)
                is_valid[i] = True
            else:
                is_valid[i] = False

        return is_valid

    def _generate_random_positions(self, num_landmarks):
        """
        randomize landmark position keeping:
        - self.xlim
        - self.ylim
        - self.rmin
        """

        positions = np.zeros((num_landmarks, 3))
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

    def print(self):
        """
        print map data
        """
        pass

    def plot_map_lims(self, color=None):
        """
        plot map limits
        """
        pass

    def plot_landmarks(self, color=None):
        """
        plot landmarks
        """
        pass

    def plot_trajectory(self, color=None):
        """
        plot landmarks
        """
        pass

    def plot_view_to_landmark_observations(self, color=None):
        """
        plot landmarks
        """
        pass

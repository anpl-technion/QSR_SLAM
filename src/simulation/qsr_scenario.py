import numpy as np


class Scenario:
    """
    2D map with landmarks
    """
    def __init__(self, rmin, xlim, ylim, azimuth_measurement_noise_std, motion_model_noise_std):
        self.xlim = xlim  # map limits [xmin,xmax]
        self.ylim = ylim  # map limits [ymin,ymax]
        self.rmin = rmin  # minimum distance between objects(landmarks and cameras)

        self.azimuth_measurement_noise_std = azimuth_measurement_noise_std * np.pi/180  # azimuth measurement noise std
        self.motion_model_noise_std = motion_model_noise_std * np.pi/180  # motion model noise std

        self.landmark_ids = np.zeros((1, 0))
        self.landmark_positions = np.zeros((0, 2))  # [x,y]

        self.view_ids = np.zeros((1, 0))
        self.view_poses = np.zeros((0, 2))  # [x,y,azimuth]

        self.view_to_landmark_observations = np.zeros((0, 4))  # [view_id, landmark_id, azimuth_gt, azimuth_measured]
        self.view_to_view_motion_model = np.zeros((0, 4))  # [view_id1, view_id2, heading_gt, heading_measured]

    def add_landmark(self, num_landmarks, landmark_ids=None, landmark_positions=None):
        """
        add landmarks to map:
        num_landmarks - number of landmarks to add
        landmark_ids - landmark ids [nx1]
                       if None landmark ids in ascending order
        landmark_positions - landmark positions [nx2]
                       if None, randomly sampled within map_lims while keeping rmin
        """

        if (num_landmarks is None) or num_landmarks < 1:
            Exception('invalid input!')

        if landmark_ids is None:
            if self.landmark_ids.shape[0] == 0:
                max_id = -1
            else:
                max_id = max(self.landmark_ids)
            landmark_ids = np.array(range(max_id + 1, max_id + 1 + num_landmarks))
        self.landmark_ids = np.hstack((self.landmark_ids, landmark_ids))

        if landmark_positions is None:
            landmark_positions = self._generate_random_positions(num_landmarks)
        self.landmark_positions = np.vstack((np.array(self.landmark_positions), np.array(landmark_positions)))

        return

    def get_landmark(self, landmark_ids):
        """
        get landmarks location
        landmark_ids - landmark ids [nx1]
        """
        landmark_positions = np.nan((landmark_ids.shape[0], 3))
        is_valid = np.array((landmark_ids.shape[0], 3), dtype=np.bool)

        for i, lid in enumerate(landmark_ids):
            if lid in self.landmark_ids:
                idx = np.argwhere(self.landmark_ids == lid)
                landmark_positions[i, :] = self.landmark_positions[idx, :]
                is_valid[i] = True
            else:
                is_valid[i] = False

        return landmark_positions, is_valid

    def remove_landmark(self, landmark_ids):
        """
        remove landmarks from map:
        landmark_ids - landmark ids [nx1]
        """
        # TODO: remove corresponding measurements
        is_valid = np.array((landmark_ids.shape[0], 3), dtype=np.bool)
        for i, lid in enumerate(landmark_ids):
            if lid in self.landmark_ids:
                idx = np.argwhere(self.landmark_ids != lid)
                self.landmark_positions = self.landmark_positions[idx, :]
                self.landmark_ids = self.landmark_ids[idx]
                is_valid[i] = True
            else:
                is_valid[i] = False

        return is_valid

    def add_view(self, num_views, view_ids=None, view_poses=None):
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

    def get_view(self, view_ids):
        """
        get view poses
        view_ids - view ids [nx1]
        """
        view_poses = np.nan((view_ids.shape[0], 3))
        is_valid = np.array((view_ids.shape[0], 3), dtype=np.bool)

        for i, lid in enumerate(view_ids):
            if lid in self.view_ids:
                idx = np.argwhere(self.view_ids == lid)
                view_poses[i, :] = self.view_poses[idx, :]
                is_valid[i] = True
            else:
                is_valid[i] = False

        return view_poses, is_valid

    def remove_view(self, view_ids):
        """
        remove view from trajectory:
        view_ids - view ids [nx1]
        """
        # TODO: remove corresponding measurements
        is_valid = np.array((view_ids.shape[0], 3), dtype=np.bool)
        for i, lid in enumerate(view_ids):
            if lid in self.view_ids:
                idx = np.argwhere(self.view_ids != lid)
                self.view_poses = self.view_poses[idx, :]
                self.view_ids = self.view_ids[idx]
                is_valid[i] = True
            else:
                is_valid[i] = False

        return is_valid

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

    def get_view_to_landmark_observations(self, view_id, landmark_id):
        """
        get view to landmark observation:
        view_id - view ids [nx1]
        landmark_id - landmark ids [nx1]
        """
        n = view_id.shape[0]
        if (view_id.shape is not (n, 1)) or (landmark_id.shape is not (n, 1)):
            Exception('invalid input!')

        is_valid = np.array((n, 3), dtype=np.bool)
        for i, vid in enumerate(view_id):
            if vid in self.view_ids and landmark_id[i] in self.landmark_ids:
                idx = np.argwhere((vid, landmark_id[i]) == self.view_to_landmark_observations(:, 2: 3))
                is_valid[i] = True
            else:
                is_valid[i] = False

        return observation, is_valid

    def remove_view_to_landmark_observations(self, view_id1, view_id2, heading):
        """
        add poses to trajectory:
        num_poses - number of poses to add
        num_poses - camera position and orientation [nx3]
                       if None, randomly sampled
        """
        pass

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

    def plot_observations(self, color=None):
        """
        plot landmarks
        """
        pass

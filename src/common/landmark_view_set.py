import numpy as np


class LandmarkViewSet:
    """
    This object holds landmarks and views
    and common measurements / motion model constraints
    """
    def __init__(self):
        self.landmark_ids = np.zeros(0)
        self.landmark_positions = np.zeros((0, 2))  # [x,y]

        self.view_ids = np.zeros(0)
        self.view_poses = np.zeros((0, 3))  # [x,y,azimuth]

        self.view_to_landmark_observations = np.zeros((0, 3))  # [view_id, landmark_id, azimuth]
        self.view_to_view_motion_model = np.zeros((0, 3))  # [view_id1, view_id2, heading]

    def add_landmark(self, num_landmarks, landmark_ids=None, landmark_positions=None):
        """
        add landmarks to map:
        num_landmarks - number of landmarks to add
        landmark_ids - landmark ids [nx1]
        landmark_positions - landmark positions [nx2] - [x,y]
        """

        if (num_landmarks is None) or num_landmarks < 1:
            Exception('invalid input!')

        if landmark_ids is None:
            Exception('invalid input!')
        landmark_ids = np.array(landmark_ids)
        self.landmark_ids = np.hstack((self.landmark_ids, landmark_ids))

        if landmark_positions is None:
            Exception('invalid input!')
        landmark_positions = np.array(landmark_positions)
        if landmark_positions.ndim == 1:
            landmark_positions = np.array([landmark_positions])
        self.landmark_positions = np.vstack((self.landmark_positions, landmark_positions))

        return

    def get_landmark(self, landmark_ids):
        """
        get landmarks location
        landmark_ids - landmark ids [nx1]
        """

        landmark_ids = np.array(landmark_ids)

        landmark_positions = np.zeros((landmark_ids.shape[0], 2))
        is_valid = np.zeros((landmark_ids.shape[0], 1), dtype=bool)
        for i, lid in enumerate(landmark_ids):
            if lid in self.landmark_ids:
                idx = np.where((self.landmark_ids == lid))
                landmark_positions[i, :] = self.landmark_positions[idx]
                is_valid[i] = True
            else:
                is_valid[i] = False

        return is_valid, landmark_positions

    def remove_landmark(self, landmark_ids):
        """
        remove landmarks from map:
        landmark_ids - landmark ids [nx1]
        """
        # TODO: remove corresponding measurements

        landmark_ids = np.array(landmark_ids)
        idx = np.logical_not(np.isin(self.landmark_ids, landmark_ids))
        self.landmark_positions = self.landmark_positions[idx]
        self.landmark_ids = self.landmark_ids[idx]

        return

    def add_view(self, num_views, view_ids=None, view_poses=None):
        """
        add views to trajectory:
        num_views - number of landmarks to add
        view_ids - view ids [nx1]
        view_poses - view poses [nx3] - [x,y,azimuth]
        """

        if (num_views is None) or num_views < 1:
            Exception('invalid input!')

        if view_ids is None:
            Exception('invalid input!')
        view_ids = np.array(view_ids)
        self.view_ids = np.hstack((self.view_ids, view_ids))

        if view_poses is None:
            Exception('invalid input!')
        view_poses = np.array(view_poses)
        if view_poses.ndim == 1:
            view_poses = np.array([view_poses])
        self.view_poses = np.vstack((self.view_poses, view_poses))

        return

    def get_view(self, view_ids):
        """
        get view poses
        view_ids - view ids [nx1]
        """

        view_ids = np.array(view_ids)

        view_poses = np.zeros((view_ids.shape[0], 3))
        is_valid = np.zeros((view_ids.shape[0], 1), dtype=bool)
        for i, lid in enumerate(view_ids):
            if lid in self.view_ids:
                idx = np.where((self.view_ids == lid))
                view_poses[i, :] = self.view_poses[idx]
                is_valid[i] = True
            else:
                is_valid[i] = False

        return is_valid, view_poses

    def remove_view(self, view_ids):
        """
        remove views from trajectory:
        view_ids - view ids [nx1]
        """
        # TODO: remove corresponding measurements
        view_ids = np.array(view_ids)
        idx = np.logical_not(np.isin(self.view_ids, view_ids))
        self.view_poses = self.view_poses[idx]
        self.view_ids = self.view_ids[idx]

        return

    def add_view_to_landmark_observations(self, view_ids=None, landmark_ids=None, azimuth=None):
        """
        add landmarks to view observations:
        view_ids - landmark ids [nx1]
        landmark_ids - landmark ids [nx1]
        data - observation data [nx1] - [azimuth]
        """

        if view_ids is None:
            Exception('invalid input!')
        view_ids = np.array(view_ids).flatten()

        if landmark_ids is None:
            Exception('invalid input!')
        landmark_ids = np.array(landmark_ids).flatten()

        if azimuth is None:
            Exception('invalid input!')
        azimuth = np.array(azimuth).flatten()

        a = np.vstack([[view_ids], [landmark_ids], [azimuth]]).T
        self.view_to_landmark_observations = np.vstack((self.view_to_landmark_observations, a))

        return

    def get_view_to_landmark_observations(self,  view_ids, landmark_ids):
        """
        get landmarks to view observations:
        landmark_ids - landmark ids [nx1]
        """

        view_ids = np.array(view_ids).flatten()
        landmark_ids = np.array(landmark_ids).flatten()
        obs_ids = np.array([view_ids, landmark_ids]).T
        is_valid = np.all(np.isin(obs_ids, self.view_to_landmark_observations[:, 0:2]))

        view_to_landmark_observations = np.zeros((landmark_ids.shape[0], 1))

        # idx = np.all(np.isin(obs_ids, self.view_to_landmark_observations[:, 0
        #                                                                     :2]))
        # view_to_landmark_observations


        view_to_landmark_observations = np.zeros((landmark_ids.shape[0], 2))
        is_valid = np.zeros((landmark_ids.shape[0], 1), dtype=bool)
        for i, lid in enumerate(landmark_ids):
            if (lid in self.landmark_ids) and (view_ids[i] in self.view_ids):

                idx = np.isin(obs_ids, self.view_to_landmark_observations[:, 0:2])
                idx = np.where(self.view_to_landmark_observations[:, 0:2] == lid)
                view_to_landmark_observations[i, :] = self.view_to_landmark_observations[idx]
                is_valid[i] = True
            else:
                is_valid[i] = False

        return view_to_landmark_observations, is_valid

    def remove_view_to_landmark_observations(self, landmark_ids):
        """
        remove landmarks to view observations:
        landmark_ids - landmark ids [nx1]
        """
        # TODO: remove corresponding measurements

        landmark_ids = np.array(landmark_ids)
        idx = np.logical_not(np.isin(self.landmark_ids, landmark_ids))
        self.landmark_positions = self.landmark_positions[idx]
        self.landmark_ids = self.landmark_ids[idx]

        return

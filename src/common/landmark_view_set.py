import numpy as np

# TODO: error if adding too many measurements
# TODO: add partial observations queries (by landmark only \ view only)
# TODO: change output to include ids


class LandmarkViewSet:
    """
    This object holds landmarks and views
    and common measurements / motion model constraints
    """
    def __init__(self, max_num_view_to_landmark, view_to_landmark_data_size,
                 max_num_view_to_view, view_to_view_data_size):
        self.landmark_ids = np.zeros(0, dtype=np.uint32)
        self.landmark_positions = np.zeros((0, 2), dtype=float)  # [x, y]

        self.view_ids = np.zeros(0, dtype=np.uint32)
        self.view_poses = np.zeros((0, 3), dtype=float)  # [x,y,azimuth]

        self.view_to_landmark_data_size = view_to_landmark_data_size
        self.max_num_view_to_landmark = max_num_view_to_landmark
        self.view_to_landmark_ids = np.zeros((self.max_num_view_to_landmark, 2), dtype=np.uint32)  # [view_id, landmark_id]
        self.view_to_landmark_data = np.zeros((self.max_num_view_to_landmark, self.view_to_landmark_data_size), dtype=float)  # [data]
        self.view_to_landmark_max_idx = 0

        self.view_to_view_data_size = view_to_view_data_size
        self.max_num_view_to_view = max_num_view_to_view
        self.view_to_view_ids = np.zeros((self.max_num_view_to_view, 2), dtype=np.uint32)  # [view_id, landmark_id]
        self.view_to_view_data = np.zeros((self.max_num_view_to_view, self.view_to_view_data_size), dtype=float)  # [data]
        self.view_to_view_max_idx = 0

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
        landmark_ids = np.array(landmark_ids, dtype=np.uint32)
        self.landmark_ids = np.hstack((self.landmark_ids, landmark_ids))

        if landmark_positions is None:
            Exception('invalid input!')
        landmark_positions = np.array(landmark_positions)
        if landmark_positions.ndim == 1:
            landmark_positions = np.array([landmark_positions])
        self.landmark_positions = np.vstack((self.landmark_positions, landmark_positions))

        return

    def get_landmark(self, landmark_ids=None):
        """
        get landmarks location
        landmark_ids - landmark ids [nx1]
        """

        if landmark_ids is None:
            landmark_ids = self.landmark_ids
        else:
            landmark_ids = np.array(landmark_ids, dtype=np.uint32)

        n = landmark_ids.shape[0]

        landmark_positions = np.zeros((n, 2))
        is_valid = np.zeros((n, 1), dtype=bool)
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

        landmark_ids = np.array(landmark_ids, dtype=np.uint32)
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
        view_ids = np.array(view_ids, dtype=np.uint32)
        self.view_ids = np.hstack((self.view_ids, view_ids))

        if view_poses is None:
            Exception('invalid input!')
        view_poses = np.array(view_poses)
        if view_poses.ndim == 1:
            view_poses = np.array([view_poses])
        self.view_poses = np.vstack((self.view_poses, view_poses))

        return

    def get_view(self, view_ids=None):
        """
        get view poses
        view_ids - view ids [nx1]
        """

        if view_ids is None:
            view_ids = self.view_ids
        else:
            view_ids = np.array(view_ids, dtype=np.uint32)

        n = view_ids.shape[0]

        view_poses = np.zeros((n, 3))
        is_valid = np.zeros((n, 1), dtype=bool)
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
        view_ids = np.array(view_ids, dtype=np.uint32)
        idx = np.logical_not(np.isin(self.view_ids, view_ids))
        self.view_poses = self.view_poses[idx]
        self.view_ids = self.view_ids[idx]

        return

    def add_view_to_landmark_observations(self, view_ids=None, landmark_ids=None, data=None):
        """
        add landmarks to view observations:
        view_ids - landmark ids [nx1]
        landmark_ids - landmark ids [nx1]
        data - observation data [nx1] - [azimuth]
        """

        if view_ids is None:
            Exception('invalid input!')
        view_ids = np.array(view_ids, dtype=np.uint32).flatten()
        n = view_ids.size
        view_ids = view_ids.reshape((n, 1))

        if landmark_ids is None:
            Exception('invalid input!')
        landmark_ids = np.array(landmark_ids, dtype=np.uint32).flatten()
        if landmark_ids.size != n:
            Exception('invalid input size!')
        landmark_ids = landmark_ids.reshape((n, 1))

        if data is None:
            Exception('invalid input!')
        data = np.array([data], dtype=float)
        if data.size != self.view_to_landmark_data_size*n:
            Exception('invalid input size!')
        data = data.reshape((n, self.view_to_landmark_data_size))

        obs_ids = np.array([view_ids, landmark_ids], dtype=np.uint32).T.reshape((n, 2))
        is_exist = np.all(np.isin(obs_ids, self.view_to_landmark_ids[0:self.view_to_landmark_max_idx, :]), 1)

        # TODO: do this more efficiently
        for i in range(0, n):
            if is_exist[i]:
                # update existing entries
                idx = np.where( np.all(self.view_to_landmark_ids[0:self.view_to_landmark_max_idx, :] == obs_ids[i, :],1) )
                self.view_to_landmark_data[idx, :] = data[i, :]

            else:
                # update new entries
                self.view_to_landmark_data[self.view_to_landmark_max_idx, :] = data[i, :]
                self.view_to_landmark_ids[self.view_to_landmark_max_idx, :] = obs_ids[i, :]
                self.view_to_landmark_max_idx = self.view_to_landmark_max_idx + 1


        # dtype = {'names': ['f1', 'f2'], 'formats': [np.uint32, np.uint32]}
        # [C, idx1, idx2] = np.intersect1d(obs_ids.view(dtype), self.view_to_landmark_ids[0:self.view_to_landmark_max_idx].view(dtype),
        #                    assume_unique=True, return_indices=True)
        # is_exist = np.zeros((n, 1), dtype=bool)
        # is_exist[idx1] = True
        #
        # # update existing entries
        # self.view_to_landmark_data[idx2] = data[idx1]
        #
        # # update new entries
        # m = idx1.size
        # is_new = np.bitwise_not(is_exist)
        # self.view_to_landmark_data[self.view_to_landmark_max_idx:self.view_to_landmark_max_idx+n-m, :] = data[is_new[:,0]]
        # self.view_to_landmark_ids[self.view_to_landmark_max_idx:self.view_to_landmark_max_idx+n-m, :] = obs_ids[is_new[:,0]]
        # self.view_to_landmark_max_idx = self.view_to_landmark_max_idx + n - m

        return is_exist

    def get_view_to_landmark_observations(self,  view_ids=None, landmark_ids=None):
        """
        get landmarks to view observations:
        landmark_ids - landmark ids [nx1]
        """

        if (view_ids is None) and (landmark_ids is None):
            n = self.view_to_landmark_ids.shape[0]
            view_ids = self.view_to_landmark_ids[:, 0].reshape((n, 1))
            landmark_ids = self.view_to_landmark_ids[:, 1].reshape((n, 1))

        elif (view_ids is None) and (landmark_ids is not None):
            Exception('not supported yet')

        elif (view_ids is not None) and (landmark_ids is None):
            Exception('not supported yet')

        else:
            view_ids = np.array(view_ids, dtype=np.uint32).flatten()
            n = view_ids.size
            view_ids = view_ids.reshape((n, 1))

            landmark_ids = np.array(landmark_ids, dtype=np.uint32).flatten()
            if landmark_ids.size != n:
                Exception('invalid input size!')
            landmark_ids = landmark_ids.reshape((n, 1))

        obs_ids = np.array([view_ids, landmark_ids], dtype=np.uint32).T.reshape((n, 2))
        is_valid = np.all(np.isin(obs_ids, self.view_to_landmark_ids[0:self.view_to_landmark_max_idx, :]), 1)

        # TODO: do this more efficiently
        view_to_landmark_data = np.zeros((n, self.view_to_landmark_data_size), dtype=float)
        view_to_landmark_data[:] = np.nan
        for i in range(0, n):
            if is_valid[i]:
                idx = np.where( np.all(self.view_to_landmark_ids[0:self.view_to_landmark_max_idx, :] == obs_ids[i, :], 1))
                view_to_landmark_data[i, :] = self.view_to_landmark_data[idx]

        # dtype = {'names': ['f1', 'f2'], 'formats': 2 * [obs_ids.dtype]}
        # [C, idx1, idx2] = np.intersect1d(obs_ids.view(dtype),
        #                                  self.view_to_landmark_ids[0:self.view_to_landmark_max_idx].view(dtype),
        #                                  assume_unique=True, return_indices=True)
        #
        # is_valid = np.zeros((n, 1), dtype=bool)
        # is_valid[idx1] = True
        #
        # view_to_landmark_observations = np.zeros((n, self.view_to_landmark_data_size), dtype=bool).fill(np.nan)
        # view_to_landmark_observations[idx1] = self.view_to_landmark_ids[idx2]

        #
        # view_ids = np.array(view_ids).flatten()
        # landmark_ids = np.array(landmark_ids).flatten()
        # obs_ids = np.array([view_ids, landmark_ids], dtype=np.uint16).T
        #
        # obs_ids2 = float(self.view_to_landmark_observations[:, 0:2])
        #
        # dtype = {'names': ['f1', 'f2'], 'formats': 2 * [obs_ids.dtype]}
        # [idx1, idx2, C] = np.intersect1d(obs_ids.view(dtype), self.view_to_landmark_observations[:, 0:2].view(dtype),
        #                    assume_unique=True, return_indices=True)
        #
        #
        # is_valid = np.all(np.isin(obs_ids, self.view_to_landmark_observations[:, 0:2]))
        #
        # view_to_landmark_observations = np.zeros((landmark_ids.shape[0], 1))
        #
        # # idx = np.all(np.isin(obs_ids, self.view_to_landmark_observations[:, 0
        # #                                                                     :2]))
        # # view_to_landmark_observations
        #
        # view_to_landmark_observations = np.zeros((landmark_ids.shape[0], 2))
        # view_to_landmark_observations.fill(np.nan)
        # is_valid = np.zeros((landmark_ids.shape[0], 1), dtype=bool)
        # for i in range(0, len(landmark_ids)):
        #
        #     view_to_landmark_observations[i, :] = np.nan
        #     is_valid[i] = False
        #     if (landmark_ids[i] in self.landmark_ids) and (view_ids[i] in self.view_ids):
        #         idx = np.all(self.view_to_landmark_observations[:, 0:2] == obs_ids, 1)
        #         if np.any(idx):
        #             view_to_landmark_observations[i, :] = self.view_to_landmark_observations[idx]
        #             is_valid[i] = True

        return view_to_landmark_data, is_valid

    def remove_view_to_landmark_observations(self, view_ids, landmark_ids):
        """
        remove landmarks to view observations:
        landmark_ids - landmark ids [nx1]
        """

        if view_ids is None:
            Exception('invalid input!')
        view_ids = np.array(view_ids, dtype=np.uint32).flatten()
        n = view_ids.size
        view_ids = view_ids.reshape((n, 1))

        if landmark_ids is None:
            Exception('invalid input!')
        landmark_ids = np.array(landmark_ids, dtype=np.uint32).flatten()
        if landmark_ids.size != n:
            Exception('invalid input size!')
        landmark_ids = landmark_ids.reshape((n, 1))

        obs_ids = np.array([view_ids, landmark_ids], dtype=np.uint32).T.reshape((n, 2))

        idx_to_remove = np.all(np.isin(self.view_to_landmark_ids[0:self.view_to_landmark_max_idx, :], obs_ids), 1)

        for i in range(self.view_to_landmark_max_idx-1, -1, -1):
            if idx_to_remove[i]:
                self.view_to_landmark_ids[i:self.view_to_landmark_max_idx-1, :] = self.view_to_landmark_ids[i+1:self.view_to_landmark_max_idx, :]
                self.view_to_landmark_ids[self.view_to_landmark_max_idx - 1, :] = 0

                self.view_to_landmark_data[i:self.view_to_landmark_max_idx-1, :] = self.view_to_landmark_data[i+1:self.view_to_landmark_max_idx, :]
                self.view_to_landmark_data[self.view_to_landmark_max_idx - 1, :] = 0

                self.view_to_landmark_max_idx = self.view_to_landmark_max_idx - 1

        return

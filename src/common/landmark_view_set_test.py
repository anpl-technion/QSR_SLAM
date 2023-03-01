import numpy as np
import common


def landmark_test():
    """
    test add / get/ remove landmarks
    """
    print('--- landmark test')

    max_num_view_to_landmark = 1000
    view_to_landmark_data_size = 1  # [azimuth]
    max_num_view_to_view = 100
    view_to_view_data_size = 1  # [heading]
    landmark_view_set = common.landmark_view_set.LandmarkViewSet(max_num_view_to_landmark, view_to_landmark_data_size,
                                                                 max_num_view_to_view, view_to_view_data_size)

    # add landmarks
    landmark_view_set.add_landmark(1, landmark_ids=[1], landmark_positions=np.array((1, 2)))
    landmark_view_set.add_landmark(3, landmark_ids=[2, 3, 4],
                                   landmark_positions=np.array(((10, 20), (11, 21), (12, 22))))
    landmark_view_set.add_landmark(2, landmark_ids=[7, 8], landmark_positions=np.array(((100, 200), (101, 201))))

    res1 = np.all(landmark_view_set.landmark_ids == np.array([1., 2., 3., 4., 7., 8.])) and \
        np.all(landmark_view_set.landmark_positions == np.array(
               [[1., 2.], [10., 20.], [11., 21.], [12., 22.], [100, 200.], [101., 201.]]))
    if res1:
        print('landmark_view_set - add landmark test PASSED!')
    else:
        print('landmark_view_set - add landmark test FAILED!')

    # get landmarks
    [lid1, lpos1] = landmark_view_set.get_landmark([1])  # get one
    [lid2, lpos2] = landmark_view_set.get_landmark([2, 3, 4])  # get multiple
    [lid3, lpos3] = landmark_view_set.get_landmark()  # get all
    res2 = np.all(lid1 == np.ones((1, 1), dtype=bool)) and \
        np.all(lpos1 == np.array([1., 2.])) and \
        np.all(lid2 == np.ones((1, 3), dtype=bool)) and \
        np.all(lpos2 == np.array([[10, 20], [11, 21], [12, 22]])) and \
        np.all(lid3 == np.ones((1, 6), dtype=bool)) and \
        np.all(lpos3 == np.array([[1., 2.], [10., 20.], [11., 21.], [12., 22.], [100, 200.], [101., 201.]]))
    if res2:
        print('landmark_view_set - get landmark test PASSED!')
    else:
        print('landmark_view_set - get landmark test FAILED!')

    # remove landmarks
    landmark_view_set.remove_landmark([1])
    landmark_view_set.remove_landmark([2, 3])
    res3 = np.all(landmark_view_set.landmark_ids == np.array([4., 7., 8.])) and \
        np.all(landmark_view_set.landmark_positions == np.array(
              [[12., 22.], [100, 200.], [101., 201.]]))
    if res3:
        print('landmark_view_set - remove landmark test PASSED!')
    else:
        print('landmark_view_set - remove landmark test FAILED!')

    return


def view_test():
    """
    test add / get/ remove views
    """
    print('--- view test')

    max_num_view_to_landmark = 1000
    view_to_landmark_data_size = 1  # [azimuth]
    max_num_view_to_view = 100
    view_to_view_data_size = 1  # [heading]
    landmark_view_set = common.landmark_view_set.LandmarkViewSet(max_num_view_to_landmark, view_to_landmark_data_size,
                                                                 max_num_view_to_view, view_to_view_data_size)
    # add views
    landmark_view_set.add_view(1, view_ids=[1], view_poses=np.array((1, 2, 3)))
    landmark_view_set.add_view\
        (3, view_ids=[2, 3, 4], view_poses=np.array(((10, 20, 30), (11, 21, 31), (12, 22, 32))))
    landmark_view_set.add_view\
        (2, view_ids=[7, 8], view_poses=np.array(((100, 200, 300), (101, 201, 301))))

    res1 = np.all(landmark_view_set.view_ids == np.array([1., 2., 3., 4., 7., 8.])) and \
        np.all(landmark_view_set.view_poses == np.array(
               [[1, 2, 3], [10, 20, 30], [11, 21, 31], [12, 22, 32], [100, 200, 300], [101, 201, 301]]))
    if res1:
        print('landmark_view_set - add view test PASSED!')
    else:
        print('landmark_view_set - add view test FAILED!')

    # get views
    [vid1, vpose1] = landmark_view_set.get_view([1])  # get one
    [vid2, vpose2] = landmark_view_set.get_view([2, 3, 4])  # get multiple
    [vid3, vpose3] = landmark_view_set.get_view()  # get all

    res2 = np.all(vid1 == np.ones((1, 1), dtype=bool)) and \
        np.all(vpose1 == np.array([1, 2, 3])) and \
        np.all(vid2 == np.ones((1, 3), dtype=bool)) and \
        np.all(vpose2 == np.array([[10, 20, 30], [11, 21, 31], [12, 22, 32]])) and \
        np.all(vid3 == np.ones((1, 6), dtype=bool)) and \
        np.all(vpose3 == np.array([[1, 2, 3], [10, 20, 30], [11, 21, 31], [12, 22, 32], [100, 200, 300], [101, 201, 301]]))
    if res2:
        print('landmark_view_set - get view test PASSED!')
    else:
        print('landmark_view_set - get view test FAILED!')

    # remove views
    landmark_view_set.remove_view([1])
    landmark_view_set.remove_view([2, 3])
    res3 = np.all(landmark_view_set.view_ids == np.array([4., 7., 8.])) and \
        np.all(landmark_view_set.view_poses == np.array(
              [[12, 22, 32], [100, 200, 300], [101, 201, 301]]))
    if res3:
        print('landmark_view_set - remove view test PASSED!')
    else:
        print('landmark_view_set - remove view test FAILED!')

    return


def view_to_landmark_observation_test():
    """
    test add / get/ remove view to landmark observation
    """
    print('--- view to landmark observation test')

    max_num_view_to_landmark = 1000
    view_to_landmark_data_size = 1  # [azimuth]
    max_num_view_to_view = 100
    view_to_view_data_size = 1  # [heading]
    landmark_view_set = common.landmark_view_set.LandmarkViewSet(max_num_view_to_landmark, view_to_landmark_data_size,
                                                                 max_num_view_to_view, view_to_view_data_size)
    # add views
    landmark_view_set.add_view\
        (3, view_ids=[1, 2, 3, 4], view_poses=np.array(((10, 20, 30), (11, 21, 31), (12, 22, 32))))

    # add landmarks
    landmark_view_set.add_landmark\
        (3, landmark_ids=[1, 2, 3, 4, 5], landmark_positions=np.array(((10, 20), (11, 21), (12, 22))))

    # add view to landmark measurements
    landmark_view_set.add_view_to_landmark_observations(view_ids=[1], landmark_ids=[1], data=[11])
    landmark_view_set.add_view_to_landmark_observations(
        view_ids=[2, 2, 3, 3, 3, 4, 4, 4, 4],
        landmark_ids=[1, 2, 1, 2, 3, 2, 3, 4, 5],
        data=[211, 221, 311, 321, 331, 421, 431, 441, 451])
    landmark_view_set.add_view_to_landmark_observations(
        view_ids=[2, 2, 3, 3, 3, 4, 4, 4, 4],
        landmark_ids=[1, 2, 1, 2, 3, 2, 3, 4, 5],
        data=[21, 22, 31, 32, 33, 42, 43, 44, 45])

    view_to_landmark_observations = np.hstack((landmark_view_set.view_to_landmark_ids[0:landmark_view_set.view_to_landmark_max_idx,:],
                                               landmark_view_set.view_to_landmark_data[0:landmark_view_set.view_to_landmark_max_idx,:]))
    res1 = np.all(np.equal(view_to_landmark_observations, np.array(
               [[1, 1, 11],
                [2, 1, 21],
                [2, 2, 22],
                [3, 1, 31],
                [3, 2, 32],
                [3, 3, 33],
                [4, 2, 42],
                [4, 3, 43],
                [4, 4, 44],
                [4, 5, 45]])))

    if res1:
        print('landmark_view_set - add view to landmark observation test PASSED!')
    else:
        print('landmark_view_set - add view to landmark observation test FAILED!')

    # get view to landmark measurements
    [az1, vobs1] = landmark_view_set.get_view_to_landmark_observations(view_ids=[1], landmark_ids=[1])
    [az2, vobs2] = landmark_view_set.get_view_to_landmark_observations(
        view_ids=[2, 2, 3, 3, 3, 4, 4, 4, 4],
        landmark_ids=[1, 2, 1, 2, 3, 2, 3, 4, 5])
    [az3, vobs3] = landmark_view_set.get_view_to_landmark_observations(
        view_ids=[22, 22, 33, 33, 43, 34, 24, 44, 34],
        landmark_ids=[1, 2, 1, 2, 3, 2, 3, 4, 5])

    res2 = np.all(vobs1 == np.ones((1, 1), dtype=bool)) and \
        np.all(az1 == np.array([11])) and \
        np.all(vobs2 == np.ones((1, 9), dtype=bool)) and \
        np.all(az2 == np.array([[21], [22], [31], [32], [33], [42], [43], [44], [45]])) and \
        np.all(vobs3 == np.zeros((1, 9), dtype=bool)) and \
        np.all(np.isnan(az3))

    if res2:
        print('landmark_view_set - get view to landmark observation test PASSED!')
    else:
        print('landmark_view_set - get view to landmark observation test FAILED!')

    # remove view to landmark measurements
    landmark_view_set.remove_view_to_landmark_observations([1], [1])
    landmark_view_set.remove_view_to_landmark_observations([2, 2], [1, 2])

    view_to_landmark_observations = np.hstack((landmark_view_set.view_to_landmark_ids[0:landmark_view_set.view_to_landmark_max_idx,:],
                                               landmark_view_set.view_to_landmark_data[0:landmark_view_set.view_to_landmark_max_idx,:]))

    res3 = np.all(np.equal(view_to_landmark_observations, np.array(
               [[3, 1, 31],
                [3, 2, 32],
                [3, 3, 33],
                [4, 2, 42],
                [4, 3, 43],
                [4, 4, 44],
                [4, 5, 45]])))
    if res3:
        print('landmark_view_set - remove view to landmark observation test PASSED!')
    else:
        print('landmark_view_set - remove view to landmark observation test FAILED!')

    return


if __name__ == "__main__":
    print('-------------- landmark_view_set test:')
    landmark_test()
    view_test()
    view_to_landmark_observation_test()
    print('-------------- landmark_view_set test Done!')



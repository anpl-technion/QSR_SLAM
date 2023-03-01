import numpy as np


if __name__ == "__main__":
    a = (0, 1)
    b = (1, 1)

    f = AbFrame(a, b)

    point = np.array([[0, 0],
                  [0, 1],
                  [1, 0],
                  [1, 1],
                  [1, 2],
                  [2, 2],
                  [2, 1],
                  [2, 0]], dtype=np.float)

    point_ab = f.transform_point_world_to_ab(point)
    point_world = f.transform_pointab_to_world(point_ab)



    pose = np.array([[0, 0, np.pi*45/180],
                    [0, 1, np.pi*45/180],
                    [1, 0, np.pi*45/180],
                    [1, 1, np.pi*45/180]], dtype=np.float)

    pose_ab = f.transform_point_world_to_ab(pose)
    pose_world = f.transform_pointab_to_world(pose_ab)



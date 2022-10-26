from dyfos_rtl.common.htransform import HTransform
import numpy as np
import octomap


class OctomapObstacleQuery:

    def __init__(self, mappath: str, mapres: float, vehicle_radius: float,
                 vehicle_height: float) -> None:
        # Loads the octomap into memory.
        self.__omap = octomap.OcTree(mapres)
        self.__omap.readBinary(mappath)
        self.__padded_radius = vehicle_radius #vehicle_radius * 2.0
        self.__padded_height = 0.3 #vehicle_height * 2.0
        self.__mapres = mapres

        # The set of "rays" used to predict collisions.
        half_pad_height = self.__padded_height / 2.
        self.__nine_direction_collision_check = np.array([
            [self.__padded_radius, 0, 0],
            [-self.__padded_radius, 0, 0],
            [0, -self.__padded_radius, 0],
            [0, self.__padded_radius, 0],
            [0, 0, -self.__padded_height],
            [-self.__padded_radius, -self.__padded_radius, -half_pad_height],
            [-self.__padded_radius, self.__padded_radius, -half_pad_height],
            [self.__padded_radius, -self.__padded_radius, -half_pad_height],
            [self.__padded_radius, self.__padded_radius, -half_pad_height]
        ])
        ones = np.ones((len(self.__nine_direction_collision_check), 1))
        self.__nine_direction_collision_check = np.concatenate((
            self.__nine_direction_collision_check, ones), axis=1)

    # end def



    def ray_check2(self, origin: np.ndarray, direction: np.ndarray) -> bool:
        # Determine if a hit occurred.
        # direction = end - origin
        # original_length = np.linalg.norm(direction)

        # Finds nearest voxel center.
        def round_to_nearest_voxel_center(point, to, places=2):
            return np.around(np.around(point / to) * to, places)

        def round_up(value, to, places=2):
            return np.around(np.ceil(value / to) * to, places)

        end_point = np.array([0., 0., 0.])
        max_range = round_up(np.linalg.norm(direction), self.__mapres)
        hit = self.__omap.castRay(
            origin=round_to_nearest_voxel_center(origin, self.__mapres),
            direction=direction,
            end=end_point,
            ignoreUnknownCells=True,
            maxRange=max_range
        )

        if hit == True:
            return True

        else:
            if np.linalg.norm(end_point - origin) < max_range:
                return True
            else:
                return False
















    def ray_check(self, origin: np.ndarray, direction: np.ndarray) -> bool:
        # Determine if a hit occurred.
        # direction = end - origin
        # original_length = np.linalg.norm(direction)

        def round_to_nearest_voxel_center(point, to, places=2):
            return np.around(np.around(point / to) * to, places)

        def round_up(value, to, places=2):
            return np.around(np.ceil(value / to) * to, places)
        max_range = round_up(np.linalg.norm(direction), self.__mapres)

        end = np.array([0., 0., 0.])
        hit = self.__omap.castRay(
            origin=round_to_nearest_voxel_center(origin, self.__mapres),
            direction=direction,
            end=end,
            ignoreUnknownCells=False,
            maxRange=max_range
        )
        return hit
        # if hit == False:
        #     return False
        # else:
        #     final_length = np.linalg.norm(end - origin)
        #     if final_length > original_length:
        #         return False
        #     else:
        #         return True

    # end def

    def check_collision(self, observer_pose: HTransform) -> bool:
        """
        Predicts if the observer will collide with an obstacles when at the
        specified pose.
        """
        # Sanity check.
        assert observer_pose.ndim() == 3

        # Transform the collision check rays so that they align with the
        # observer.
        obs_rot = np.eye(4)
        obs_rot[:3, :3] = observer_pose.matrix()[:3,:3]
        tfd_rays = np.matmul(obs_rot, self.__nine_direction_collision_check.T).T

        # Check if any ray intersects with an obstacle in the octomap. Performs
        # a short-curcuit check where if any ray predicts a collision, the
        # function returns True immediately.
        observer_position = observer_pose.tvec()

        for ray_end in tfd_rays:

            if self.ray_check2(observer_position, ray_end[:3]):
                return True
        return False

    # end def

    def check_occlusion(self, observer_pose: HTransform,
                        target_pose: HTransform, target_radius: float) -> bool:
        """
        Predicts if the target (with position and radius) will be occluded by
        an obstacle when the observer is at the specified pose.
        """
        # Compute where the target will be in observer's frame.
        obs_ht_inv = observer_pose.inverse_matrix().matrix()
        target_position = np.array([target_pose.tvec().tolist() + [1]])
        target_pose_of = np.matmul(obs_ht_inv, target_position.T)

        # Compute the two rays used to predict occlusions in the world frame.
        ray_end_offset = np.array([
            [0., -target_radius, 0., 0.],
            [0., -target_radius / 2, 0., 0.],
            [0., 0., 0., 0.],
            [0., target_radius / 2., 0., 0.],
            [0., target_radius, 0., 0.]
        ]).T
        ray_ends = np.matmul(observer_pose.matrix(),
                             target_pose_of + ray_end_offset)
        ray_ends = np.transpose(ray_ends[:3])

        # Predict if the target will be occluded.
        obs_pos = observer_pose.tvec()
        for ray_end in ray_ends:
            hit = self.ray_check(obs_pos, ray_end.flatten())
            if hit == True:
                return True
        # end for
        return False
    # end def
# end class
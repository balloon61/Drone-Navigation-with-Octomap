#!/usr/bin/env python

import rospy

from mavros_px4_vehicle.experimental.avoidance import FlightAvoidance
from mavros_px4_vehicle.px4_offboard_modes import SetPositionWithYawCmdBuilder
from mavros_px4_vehicle.px4_modes import PX4_MODE_OFFBOARD
from mavros_px4_vehicle.px4_vehicle import PX4Vehicle

import sys

sys.path.insert(0, '/home/po-lun/repos/dyfos-ros-target-localization/src')

from dyfos_rtl.common.htransform import HTransform
from dyfos_rtl.search.omap_query import OctomapObstacleQuery
import numpy as np
import os
import rospkg
import random


def offboard_avoidance_vertical():
    # Create a copter instance and arm the copter.
    rospy.loginfo("Connecting to the vehicle.")
    copter = PX4Vehicle(auto_connect=True)
    copter.arm()
    copter.wait_for_status(copter.is_armed, True, 2)

    # Request the copter to hover using local position commands.
    rospy.loginfo("Sending the set position commands.")
    cmd = SetPositionWithYawCmdBuilder.build(z=2.)
    copter.set_pose2d(cmd)
    copter.sleep(2)

    rospy.loginfo("Changing to offboard mode.")
    copter.set_mode(PX4_MODE_OFFBOARD)
    copter.sleep(10.)

    observer_radius = float(rospy.get_param("radius", default=0.5))
    observer_height = float(rospy.get_param("height", default=0.3))

    # Get the path to the world directory.
    rospack = rospkg.RosPack()
    gazebo_world_dir = os.path.join(rospack.get_path('dyfos_target_localize'), "worlds/sim/gazebo")

    # Load the world XML file to find all the cylinders.
    mappath = os.path.join(gazebo_world_dir, "random_cylinders.bt").encode()
    oquery = OctomapObstacleQuery(mappath=mappath, mapres=0.05, vehicle_radius=observer_radius,
                                  vehicle_height=observer_height)
    free_space = list()
    while len(free_space) < 5:
        x, y = random.uniform(-15, 15), random.uniform(-15, 15)
        ht = HTransform(3).set_translation([x, y, 2.0])
        omap_collision = oquery.check_collision(ht)
        if omap_collision == False:
            free_space.append([x, y, 2., 0.])
    for goal in free_space:
        rospy.loginfo("Flying to location {} with yaw {}".format(goal[:3], goal[-1]))
        avoidance = FlightAvoidance(px4_vehicle=copter, min_safe_altitude=15.5,
                                    max_safe_altitude=15.7, avoid_mode=FlightAvoidance.AVOID_MODE_VERTICAL)

        avoidance.go_to(goal)

        rospy.loginfo("Reached goal.")
        rospy.sleep(2.)

    rospy.loginfo("Landing the copter.")
    copter.land(block=True)
    if copter.is_armed():
        copter.disarm()
    copter.disconnect()


# end def

if __name__ == "__main__":
    rospy.init_node("offboard_avoidance_vertical")
    offboard_avoidance_vertical()
    rospy.spin()

#!/usr/bin/env python3

import rospy
import numpy as np
import math

from autoware_mini.msg import Path
from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import VehicleCmd
from shapely.geometry import LineString, Point
from shapely import prepare, distance
from tf.transformations import euler_from_quaternion

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.path_linestring = None
        # Reading in the parameter values
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

        self.vehicle_cmd = VehicleCmd()
        self.vehicle_cmd.ctrl_cmd.steering_angle = 0.2
        self.vehicle_cmd.ctrl_cmd.linear_velocity = 10.0

    def path_callback(self, msg):
        # TODO
        # convert waypoints to shapely linestring
        self.path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(self.path_linestring)

    def current_pose_callback(self, msg):
        # TODO
        if self.path_linestring is None:
            return
        
        self.vehicle_cmd.header.stamp = msg.header.stamp
        self.vehicle_cmd.header.frame_id = 'base_link'
        self.vehicle_cmd_pub.publish(self.vehicle_cmd)

        current_pose = Point([msg.pose.position.x, msg.pose.position.y])
        d_ego_from_path_start = self.path_linestring.project(current_pose)

        # using euler_from_quaternion to get the heading angle

        _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        print("heading angle", heading)

        lookahead_point =  self.path_linestring.interpolate(d_ego_from_path_start + self.lookahead_distance)
        
        # lookahead point heading calculation
        lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)
        print("lookahead heading", lookahead_heading)
        
        alpha = lookahead_heading - heading

        print("alpha", alpha)

        ld = distance(current_pose, lookahead_point)

        steering_angle = np.arctan((2*self.wheel_base*math.sin(alpha))/ld)

        self.vehicle_cmd.ctrl_cmd.steering_angle = steering_angle


 
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
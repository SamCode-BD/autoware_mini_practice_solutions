#!/usr/bin/env python3

import rospy
import numpy as np
import math

# All these imports from lanelet2 library should be sufficient
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
from autoware_mini.lanelet2 import load_lanelet2_map
from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path, Waypoint
from autoware_mini.path import PathWrapper
import shapely
 



class Lanelet2GlobalPlanner:
    def __init__(self):
        
        #Parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")
        self.distance_to_goal_limit = rospy.get_param("lanelet2_global_planner/distance_to_goal_limit")
        #self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit")
        self.speed_limit = rospy.get_param("global_planner/speed_limit")
        #self.ego_vehicle_stopped_speed_limit = rospy.get_param("ego_vehicle_stopped_speed_limit")
        #self.lanelet_search_radius = rospy.get_param("~lanelet_search_radius")
        #self.waypoint_interval = rospy.get_param("waypoint_interval")
        #self.routing_cost = rospy.get_param("~routing_cost")

        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                lanelet2.traffic_rules.Participants.VehicleTaxi)
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        #class variables
        self.current_location = None
        self.goal_point = None

        #Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Path, queue_size=10, latch=True)

        #Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=None)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def goal_callback(self, msg):
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
            msg.pose.orientation.w, msg.header.frame_id)
        
        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        
        if self.current_location is None:
            # TODO handle if current_pose gets lost at later stage - see current_pose_callback
            rospy.logwarn("%s - current_pose not available", rospy.get_name())
            return

        start_point = shapely.Point(self.current_location.x, self.current_location.y)
        new_goal = shapely.Point(self.goal_point.x, self.goal_point.y)

        # get start and end lanelets
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]
        # find routing graph
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)
        
        if route is None:
            rospy.logerr("%s - no route found, try new goal!", rospy.get_name())
            return
        
        # find shortest path
        path = route.shortestPath()
        # This returns LaneletSequence to a point where a lane change would be necessary to continue
        path_no_lane_change = path.getRemainingLane(start_lanelet)
        print(path_no_lane_change)

        waypoints = self.lanelet_to_waypoint(path_no_lane_change)

        global_path = PathWrapper(waypoints, velocities=True, blinkers=True)

        # Find distance to start and goal waypoints
        start_point_distance = global_path.linestring.project(start_point)
        new_goal_point_distance = global_path.linestring.project(new_goal)

        # Interpolate point coordinates
        new_goal_on_path = global_path.linestring.interpolate(new_goal_point_distance)

        # Update member variables
        self.goal_point = new_goal_on_path
        
        # Trim the global path 
        trimmed_waypoints = global_path.extract_waypoints(start_point_distance, new_goal_point_distance, trim=True, copy=True)

        self.waypoint_pub(trimmed_waypoints)

    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        if self.goal_point != None:
            d = math.sqrt((self.current_location.x - self.goal_point.x)*(self.current_location.x - self.goal_point.x)  
                    + (self.current_location.y - self.goal_point.y)*(self.current_location.y - self.goal_point.y))
            if d < self.distance_to_goal_limit:
                waypoints = []
                self.waypoint_pub(waypoints)
                rospy.loginfo("%s - Goal has been reached, path cleared", rospy.get_name())
                return


    def lanelet_to_waypoint(self, lanelet_sequence):
        waypoints = []
        last_lanelet = False

        for i, lanelet in enumerate(lanelet_sequence):
            if i == len(lanelet_sequence)-1:
                last_lanelet = True

            # code to check if lanelet has attribute speed_ref
            if 'speed_ref' in lanelet.attributes:
                speed = float(lanelet.attributes['speed_ref'])
                speed = speed/3.6
            else:
                speed = self.speed_limit/3.6
    
            # Loop over the current lanelet's centerline points
            for idx, point in enumerate(lanelet.centerline):

                if not last_lanelet and idx == len(lanelet.centerline)-1:
                    # Skip last point on every lanelet (except last), because it is the same as the first point of the following lanelet
                    break

                # create Waypoint (from autoware_mini.msgs import Waypoint) and get the coordinats from lanelet.centerline points
                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed
                waypoints.append(waypoint)
            

        return waypoints

    def waypoint_pub(self, waypoints):
        path = Path()        
        path.header.frame_id = self.output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = waypoints
        self.waypoints_pub.publish(path)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()
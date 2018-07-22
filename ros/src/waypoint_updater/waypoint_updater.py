#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.loginfo('Initializing WaypointUpdater.')
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint
        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()  # Gives control over publishing frequency.
        
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoints_2d and self.waypoint_tree:
                # Get closest waypoint
                self.publish_waypoints(self.get_closest_waypoint_idx())
            rate.sleep()
            
    def get_closest_waypoint_idx(self):
        rospy.loginfo('Called get_closest_waypoint_idx().')
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # .query returns (distance, index)
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        prev_idx = closest_idx - 1  # Python slicing allows negative values.
        
        # Equation for hyperplane through closest_coords
        closest_vect = np.array(self.waypoints_2d[closest_idx])
        prev_vect = np.array(self.waypoints_2d[prev_idx])
        pos_vect = np.array([x, y])
        
        val = np.dot(closest_vect - prev_vect, pos_vect - closest_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    def publish_waypoints(self, closest_idx):
        rospy.loginfo('Called publish_waypoints().')
        lane = Lane()
        lane.header = self.base_waypoints.header
        # Python slicing takes care of potential out-of-bounds errors.
        lane.waypoints = self.base_waypoints.waypoints[
            closest_idx : closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
        
    def pose_cb(self, msg):
        rospy.loginfo('Called pose_cb().')
        self.pose = msg

    def waypoints_cb(self, waypoints):
        rospy.loginfo('Called waypoints_cb().')
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [
                [w.pose.pose.position.x, w.pose.pose.position.y]
                for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

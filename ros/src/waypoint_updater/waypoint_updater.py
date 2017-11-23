#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
from std_msgs.msg import Int32
import time
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

# Implentation parameters
LOOKAHEAD_WPS = 200    # Number of waypoints we will publish
PUBLISH_RATE = 20      # Publishing rate (Hz)

max_wp_distance = 20.0 
debugging = False               

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.base_waypoints_viz_pub = rospy.Publisher('base_waypoints_viz', Path, queue_size=2)
        self.final_waypoints_viz_pub = rospy.Publisher('final_waypoints_viz', Path, queue_size=2)
        # State variables
        self.base_waypoints = [] 
        self.base_vels = []
        self.next_waypoint = None
        self.current_pose = None
        self.red_light_waypoint = None 
        self.msg_seq = 0

        self.accel = max(rospy.get_param('/dbw_node/decel_limit') *0.5, -1.0)
        self.stop_distance = rospy.get_param('~stop_distance', 5.0)  # Distance (m) where car will stop before red light
        
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            self.update_and_publish()
            rate.sleep()

    def _update_next_waypoint(self):
        if not self.base_waypoints:
            return False

        if not self.current_pose:
            return False

        ego_x = self.current_pose.position.x
        ego_y = self.current_pose.position.y
        ego_theta = math.atan2(self.current_pose.orientation.y, self.current_pose.orientation.x)

        t = time.time()
        wp = None
        yaw = 0
        dist = 1000000 # Long number
        if self.next_waypoint:
            idx_offset = self.next_waypoint
            full_search = False
        else:
            idx_offset = 0
            full_search = True
        num_base_wp = len(self.base_waypoints)

        for i in range(num_base_wp):
            idx = (i + idx_offset)%(num_base_wp)
            wp_x = self.base_waypoints[idx].pose.pose.position.x
            wp_y = self.base_waypoints[idx].pose.pose.position.y
            wp_d = math.sqrt((ego_x - wp_x)**2 + (ego_y - wp_y)**2)

            if wp_d < dist:
                dist = wp_d
                wp = idx
                if debugging:
                    # Angle betwee car heading and waypoint heading
                    yaw = math.atan2(wp_y - ego_y, wp_x - ego_x) - ego_theta
            elif not full_search:
                # Local minimum. If the waypoint makes sense, just use it and break
                if dist < max_wp_distance:
                    break; # We found a point
                else:
                    # We seem to have lost track. We search again
                    rospy.logwarn("Waypoint updater lost track (local min at %.1f m after %d waypoints). Going back to full search.", dist, i+1)
                    full_search = True

        if wp is None:
            rospy.logwarn("Waypoint updater did not find a valid waypoint")
            return False

        self.next_waypoint = wp
        return True

    def update_and_publish(self):
        """
        - Update next_waypoint based on current_pose and base_waypoints
        - Generate the list of the next LOOKAHEAD_WPS waypoints
        - Update velocity for them
        - Publish them to "/final_waypoints"
        """
        # 1. Find next_waypoint based on ego position & orientation
        if self._update_next_waypoint():

            # 2. Generate the list of next LOOKAHEAD_WPS waypoints
            num_base_wp = len(self.base_waypoints)
            last_base_wp = num_base_wp-1
            waypoint_idx = [idx % num_base_wp for idx in range(self.next_waypoint,self.next_waypoint+LOOKAHEAD_WPS)]
            final_waypoints = [self.base_waypoints[wp] for wp in waypoint_idx]

            # Start from original velocities
            for idx in waypoint_idx:
                self.set_waypoint_velocity(self.base_waypoints, idx, self.base_vels[idx])
            try:
                red_idx = waypoint_idx.index(self.red_light_waypoint)
                self.decelerate(final_waypoints, red_idx, self.stop_distance)
            except ValueError:
                # No red light available: self.red_light_waypoint is None or not in final_waypoints
                red_idx = None

            # If we are close to the end of the circuit, make sure that we stop there
            if self.base_vels[-1] < 1e-5:
                try:
                    last_wp_idx = waypoint_idx.index(last_base_wp)
                    self.decelerate(final_waypoints, last_wp_idx, 0)
                except ValueError:
                    pass
            total_vel = 0
            for fwp in final_waypoints:
                total_vel+=fwp.twist.twist.linear.x
                
            self.publish(final_waypoints)


    def publish(self, waypoints):
            lane = Lane()
            path = Path()
            path.header.frame_id = '/world'
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            path.header.stamp = rospy.Time.now()
            lane.waypoints = waypoints
            path.poses = [x.pose for x in waypoints]
            self.final_waypoints_pub.publish(lane)
            self.final_waypoints_viz_pub.publish(path)
            self.msg_seq += 1

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):

        t = time.time()
        waypoints = msg.waypoints
        num_wp = len(waypoints)

        self.base_vels = [self.get_waypoint_velocity(waypoints, idx) for idx in range(num_wp)]
        self.base_waypoints = waypoints

        # Visualizer for base waypoints in Autoware, rviz must be launch first before this node
        path = Path()
        path.header.frame_id = '/world'
        path.header.stamp = rospy.Time(0)
        path.poses = [x.pose for x in waypoints]
        self.base_waypoints_viz_pub.publish(path)

        self.base_wp_sub.unregister()

    def traffic_cb(self, msg):
        """
        Receive and store the waypoint index for the next red traffic light.
        If the index is <0, then there is no red traffic light ahead
        """
        prev_red_light_waypoint = self.red_light_waypoint
        self.red_light_waypoint = msg.data if msg.data >= 0 else None
        if prev_red_light_waypoint != self.red_light_waypoint:
            if debugging:
                rospy.loginfo("TrafficLight changed: %s", str(self.red_light_waypoint))
            self.update_and_publish() # Refreshing on light change

    def obstacle_cb(self, msg):
        # TODO: ?
        pass


    def decelerate(self, waypoints, stop_index, stop_distance):
        """
        Decelerate a list of wayponts so that they stop on stop_index
        """
        if stop_index <= 0:
            return
        dist = self.distance(waypoints, 0, stop_index)
        step = dist / stop_index
        
        v = 0.
        d = 0.
        for idx in reversed(range(len(waypoints))):
            if idx < stop_index:
                d += step
                if d > self.stop_distance:
                    v = math.sqrt(2*abs(self.accel)*(d-stop_distance))
                    if v < 0.01:
                        v = 0
            if v < self.get_waypoint_velocity(waypoints, idx):
                self.set_waypoint_velocity(waypoints, idx, v)

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x


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
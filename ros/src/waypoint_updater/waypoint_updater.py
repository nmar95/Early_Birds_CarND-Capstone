#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import Int32

import math
import numpy as np
from scipy.spatial import distance
from itertools import islice, cycle

DEBUG = False              # get printout

NO_LIGHT = -10000000

DISTANCE_LIGHT_IGNORE   = 100.0 # start slowing down at this distance
DISTANCE_LIGHT_GREEN_GO = 10.0  # if light turns green and we are within this distance, go full speed ahead

# we have these car states:
NO_LIGHT_IN_SIGHT           = 0
STOPPING_FOR_LIGHT          = 1
ACCELERATING_THROUGH_LIGHT  = 2

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
                    # NOTE: we will only check for red lights within this limit..


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.base_waypoints_viz_pub = rospy.Publisher('base_waypoints_viz', Path, queue_size=2)


        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.final_waypoints_viz_pub = rospy.Publisher('final_waypoints_viz', Path, queue_size=2)

        # TODO: Add other member variables you need below
        self.velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))  # speed limit !
        self.decel_limit = rospy.get_param('/dbw_node/decel_limit', -5)
        
        self.base     = None   # base waypoints
        
        self.next_light_stopline_wp_id = NO_LIGHT
        self.light_is_red = False
        
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0        

        self.car_state = None
        self.previous_light = None

        rospy.spin()

    def pose_cb(self, msg):
        
        if self.base is None:
            rospy.logwarn("waypoint_updater: waypoints not yet loaded\n Maybe need to start waypoint_loader by hand?")            
            return  # don't have base yet. 
        
        #
        # Logic:
        # - Find the closest waypoint to car p1
        # - Next waypoint p2 from list of waypoints
        # - Calculate dot product of these vectors:
        #     v1: p1->p2
        #     v2: p1->car
        # - If dot product is positive then p2 is next waypoint else p1
        
        car_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
        car_pos = np.array( [car_pos] ) # 2D array, valid for cdist
        dist    = distance.cdist(self.base_pos, car_pos)
        i1      = np.argmin(dist)
        if i1 == len(self.base_pos)-1:
            i2 = 0
        else:
            i2 = i1+1      
        car_pos = car_pos.flatten() # get rid of 2nd dimension
        
        p1 = self.base_pos[i1]
        p2 = self.base_pos[i2]    
        v1 = p2      - p1
        v2 = car_pos - p1
        dot_product = np.dot(v1,v2)
        
        if dot_product > 0: # i1 is behind car, so pick next one as start of final_waypoints
            i1 = i2
        
        if self.car_state is None and self.next_light_stopline_wp_id != NO_LIGHT:
            if i1 > self.next_light_stopline_wp_id:
                if DEBUG:
                    rospy.logwarn("waypoint_updater: i1, stopline (NOT VALID YET) = %i, %i",
                                  i1, self.next_light_stopline_wp_id)
                    
                return # non-valid light message. Don't publish anything yet...
            
        if self.next_light_stopline_wp_id != NO_LIGHT:
            
            if self.next_light_stopline_wp_id == self.previous_light:
                same_light = True
            else:
                same_light = False
                self.previous_light = self.next_light_stopline_wp_id
                
            wps_to_stopline = list(islice(cycle(self.base.waypoints), i1, self.next_light_stopline_wp_id - 1))
            distance_to_stopline = self.distance(wps_to_stopline,0,len(wps_to_stopline)-1)
            #if DEBUG:
            #    rospy.logwarn("waypoint_updater: current_velocity, stop_line_wp_id, distance= %f, %s, %f",
            #                  self.current_linear_velocity, str(self.next_light_stopline_wp_id), distance_to_stopline)
        
            if DEBUG:
                rospy.logwarn("waypoint_updater: i1, stopline, is_red, distance = %i, %i, %i, %f",
                              i1, self.next_light_stopline_wp_id, self.light_is_red, distance_to_stopline)
                
            if distance_to_stopline>DISTANCE_LIGHT_IGNORE:
                self.car_state = NO_LIGHT_IN_SIGHT
                final_waypoints = list(islice(cycle(self.base.waypoints), i1, i1 + LOOKAHEAD_WPS - 1))
                final_waypoints = self.drive_constant_speed(final_waypoints, self.velocity) 
                if DEBUG:
                    rospy.logwarn("waypoint_updater: No light in sight")                
                    
            elif (distance_to_stopline<DISTANCE_LIGHT_GREEN_GO and self.light_turned_green): # Only accelerate through if it just now turned green....
                self.car_state = ACCELERATING_THROUGH_LIGHT
                final_waypoints = list(islice(cycle(self.base.waypoints), i1, i1 + LOOKAHEAD_WPS - 1))
                final_waypoints = self.drive_constant_speed(final_waypoints, self.velocity) 
                if DEBUG:
                    rospy.logwarn("waypoint_updater: Accelerating through light")                
            
            elif (self.car_state == ACCELERATING_THROUGH_LIGHT and same_light):
                self.car_state = ACCELERATING_THROUGH_LIGHT
                final_waypoints = list(islice(cycle(self.base.waypoints), i1, i1 + LOOKAHEAD_WPS - 1))
                final_waypoints = self.drive_constant_speed(final_waypoints, self.velocity) 
                if DEBUG:
                    rospy.logwarn("waypoint_updater: Finish Accelerating through light")                                
                    
            else:
                self.car_state = STOPPING_FOR_LIGHT
                # waypoints up to the stopline + 5
                ADD = 5
                final_waypoints = list(islice(cycle(self.base.waypoints), i1, self.next_light_stopline_wp_id - 1 + ADD)) # add 5 waypoints passed stopping line
                
                for i in range(len(final_waypoints)):
                    remaining = len(final_waypoints)-1 - i
                    if remaining < 10+ADD: # Note we added waypoints passed stopping line.
                        vel = 0.0
                    elif remaining < 15+ADD:
                        vel = 0.25
                    elif remaining < 20+ADD:
                        vel = 0.5
                    else:
                        vel = 1.0
                        
                    final_waypoints[i].twist.twist.linear.x = vel
                if DEBUG:
                    rospy.logwarn("waypoint_updater: Stopping for light")
                    #self.print_waypoints_velocity(final_waypoints)                        
                    
        else:
            self.car_state == NO_LIGHT_IN_SIGHT
            
            final_waypoints = list(islice(cycle(self.base.waypoints), i1, i1 + LOOKAHEAD_WPS - 1))
            final_waypoints = self.drive_constant_speed(final_waypoints, self.velocity)             
            
            
        self.publish(final_waypoints)
            
        
    def waypoints_cb(self, msg):
        """
        Called once: provides waypoints of base
        
        The msg contains a styx_msgs/Lane for all the waypoints of the base.
        """
        
        # store waypoints data in numpy arrays
        base_pos = []     # Position:                  x, y, z
        base_ori = []     # Orientation:               x, y, z, w
        base_vel = []     # Velocity in car direction: x, y, z  (only x non-zero)
        base_ang = []     # Angular velocity:          x, y, z
        for wp in msg.waypoints:
            wp_pos = [wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z]
            wp_ori = [wp.pose.pose.orientation.x, wp.pose.pose.orientation.y, wp.pose.pose.orientation.z, wp.pose.pose.orientation.w]
            wp_vel = [wp.twist.twist.linear.x, wp.twist.twist.linear.y, wp.twist.twist.linear.z]
            wp_ang = [wp.twist.twist.angular.x, wp.twist.twist.angular.y, wp.twist.twist.angular.z]
            
            base_pos.append(wp_pos)
            base_ori.append(wp_ori)
            base_vel.append(wp_vel)
            base_ang.append(wp_ang)
            
        self.base_pos = np.array(base_pos)
        self.base_ori = np.array(base_ori)
        self.base_vel = np.array(base_vel)
        self.base_ang = np.array(base_ang)

        # Visualizer for base waypoint in Autoware
        path = Path()
        path.header.frame_id = '/world'
        path.header.stamp = rospy.Time(0)
        path.poses = [x.pose for x in msg.waypoints]
        self.base_waypoints_viz_pub.publish(path)
        # store base_waypoints
        self.base = msg
                       
    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        if msg.data == NO_LIGHT:
            self.next_light_stopline_wp_id = NO_LIGHT
        else:
            self.next_light_stopline_wp_id = abs(msg.data)
            if msg.data > 0:
                self.light_is_red       = True
                self.light_turned_green = False
            else:
                if self.light_is_red == True:
                    self.light_turned_green = True
                else:
                    self.light_turned_green = False
                
                self.light_is_red = False

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
    
    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)
    
    def drive_constant_speed(self, waypoints, speed):
        for wp in waypoints:
            wp.twist.twist.linear.x = speed
        return waypoints
                
    def publish(self, waypoints):   # NOTE: copied from waypoint_loader
        lane = Lane()
        path = Path()
        path.header.frame_id = '/world'
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        path.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        path.poses = [x.pose for x in waypoints]
        self.final_waypoints_pub.publish(lane)
        self.final_waypoints_viz_pub.publish(path)

    def print_waypoints_velocity(self, waypoints):
        rospy.logwarn("waypoint_updater, waypoint velocities:\n")
        last_id = len(waypoints)-1 
        for i in range(last_id,-1,-1):
            wp = waypoints[i]
            vel=wp.twist.twist.linear.x
            rospy.logwarn("%f, ",vel)
            
    def current_velocity_cb(self, msg):
        # msg type: TwistStamped
        # Provides update on current velocity
        self.current_linear_velocity = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.z
        pass     

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

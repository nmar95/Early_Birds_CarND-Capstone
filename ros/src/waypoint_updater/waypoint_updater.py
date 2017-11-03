#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
from scipy.spatial import distance
from itertools import islice, cycle

DEBUG = False              # get printout

RED_LIGHT_DECEL = 0.05 # lower this number to stop car quicker at a light
RED_LIGHT_CUTOFF= 0.1
DISTANCE_LIGHT_IGNORE = 75.0

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

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))  # speed limit !
        self.decel_limit = rospy.get_param('/dbw_node/decel_limit', -5)
        
        self.base     = None   # base waypoints
        
        self.red_light_stopline_wp_id = -1

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if DEBUG:
            rospy.logwarn("===========================================================\n \
                           Entered callback pose_cb \n")
        
        if self.base is None:
            return  # don't have base yet. 
        
        #
        # Logic:
        # - Find the closest waypoint to car p1
        # - Next waypoint p2 from list of waypoints
        # - Calculate dot product of these vectors:
        #     v1: p1->p2
        #     v2: p1->car
        # - If dot product is positive then p2 is next waypoint else p1
        
        car_pos = np.array( [[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]] ) # 2D array, valid for cdist
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
        
        if self.red_light_stopline_wp_id != -1:
            wps_to_stopline = list(islice(cycle(self.base.waypoints), i1, self.red_light_stopline_wp_id - 1))
            distance_to_stopline = self.distance(wps_to_stopline,0,len(wps_to_stopline)-1)
            if DEBUG:
                rospy.logwarn("waypoint_updater: stop_line_wp_id, distance= %s, %f",str(self.red_light_stopline_wp_id), distance_to_stopline)
        
        if self.red_light_stopline_wp_id == -1 or distance_to_stopline>DISTANCE_LIGHT_IGNORE:
            # free & clear, drive at speed limit
            final_waypoints = list(islice(cycle(self.base.waypoints), i1, i1 + LOOKAHEAD_WPS - 1))
            final_waypoints = self.drive_at_speed_limit(final_waypoints)        
        else:
            # stop at light waypoint for stopping line
            final_waypoints = list(islice(cycle(self.base.waypoints), i1, self.red_light_stopline_wp_id - 1 ))
            if len(final_waypoints) == 0 or len(final_waypoints) > LOOKAHEAD_WPS : # this can happen if car is at or past stopline already...
                final_waypoints = list(islice(cycle(self.base.waypoints), i1, i1 + 3 ))
            final_waypoints = self.decelerate(final_waypoints)
            # DEBUG
            #self.print_waypoints_velocity(final_waypoints)
            
        self.publish(final_waypoints)
            
        if DEBUG: 
            rospy.logwarn("Leaving callback pose_cb \n \
                           ===========================================================\n")        

    def waypoints_cb(self, msg):
        # TODO: Implement
        """
        Called once: provides waypoints of base
        
        The msg contains a styx_msgs/Lane for all the waypoints of the base.
        """
        if DEBUG:        
            rospy.logwarn("===========================================================\n \
                           Entering callback waypoints_cb \n") 
            rospy.logwarn("# of waypoints = %s",len(msg.waypoints))
        
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
        
        # store base_waypoints
        self.base = msg
        
        if DEBUG:   
            rospy.logwarn("Leaving callback waypoints_cb \n \
                           ===========================================================\n") 
                       
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement    
        self.red_light_stopline_wp_id = msg.data # -1: no red light

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
    
    def drive_at_speed_limit(self, waypoints):
        for wp in waypoints:
            wp.twist.twist.linear.x = self.velocity
        return waypoints
                
    def decelerate(self, waypoints):  # NOTE: copied from waypoint_loader
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        last_id = len(waypoints)-1 
        for i in range(last_id-1,-1,-1):
            wp = waypoints[i]
            dist = self.distance(waypoints, i, last_id)
            vel = math.sqrt(2.0 * RED_LIGHT_DECEL * dist)
            vel = min(self.velocity, vel) # limit it to the speed-limit
            if vel < RED_LIGHT_CUTOFF:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)  # TODO: Is this OK??
        return waypoints

    def publish(self, waypoints):   # NOTE: copied from waypoint_loader
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def print_waypoints_velocity(self, waypoints):
        rospy.logwarn("waypoint_updater, waypoint velocities:\n")
        for wp in waypoints:
            vel=wp.twist.twist.linear.x
            rospy.logwarn("%f, ",vel)
        rospy.logwarn("\n")

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

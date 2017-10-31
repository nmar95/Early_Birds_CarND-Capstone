#!/usr/bin/env python

AB_DEBUG = False

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np
from scipy.spatial import distance
from itertools import islice, cycle

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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base     = None   # base waypoints
        self.final    = None   # final waypoints
        self.final_i1 = None   # index of first final waypoint in base waypoints

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if AB_DEBUG:
            rospy.logwarn("===========================================================\n \
                           Entered callback pose_cb \n")
            #rospy.logwarn("W0: msg = %s",msg)
        
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
        
        if dot_product < 0:
            self.final_i1 = i1
        else:
            self.final_i1 = i2
        
        final_waypoints = list(islice(cycle(self.base.waypoints), self.final_i1, self.final_i1 + LOOKAHEAD_WPS - 1))
        
        velocity = 5.0 # for testing
        for waypoint in final_waypoints:
            waypoint.twist.twist.linear.x = velocity
            
        lane = Lane()
        lane.waypoints = final_waypoints
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)
        
        self.final = lane
            
            
        
        if AB_DEBUG: 
            rospy.logwarn("Leaving callback pose_cb \n \
                           ===========================================================\n")        

    def waypoints_cb(self, msg):
        # TODO: Implement
        """
        Called once: provides waypoints of base
        
        The msg contains a styx_msgs/Lane for all the waypoints of the base.
        """
        if AB_DEBUG:        
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
        
        if AB_DEBUG:   
            rospy.logwarn("Leaving callback waypoints_cb \n \
                           ===========================================================\n") 
                  

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

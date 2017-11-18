#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
from scipy.spatial import distance
import threading

DEBUG = True              # get printout

USE_SIMULATOR_STATE = False # For testing: use simulator provided topic /vehicle/traffic_lights
STATE_COUNT_THRESHOLD = 3

NO_LIGHT = -10000000

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.bridge = CvBridge()

        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.isClassifierAvailable = True
        self.async_light_state = TrafficLight.UNKNOWN

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        """
        Called once: provides waypoints of base
    
        The msg contains a styx_msgs/Lane for all the waypoints.
        """  
        # store waypoints data in numpy arrays
        base_pos   = []     # Position:                  x, y, z
        base_posXY = []     # Position, but only XY:     x, y
        base_ori   = []     # Orientation:               x, y, z, w
        base_vel   = []     # Velocity in car direction: x, y, z  (only x non-zero)
        base_ang   = []     # Angular velocity:          x, y, z
        for wp in msg.waypoints:
            wp_pos   = [wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z]
            wp_posXY = [wp.pose.pose.position.x, wp.pose.pose.position.y]
            wp_ori   = [wp.pose.pose.orientation.x, wp.pose.pose.orientation.y, wp.pose.pose.orientation.z, wp.pose.pose.orientation.w]
            wp_vel   = [wp.twist.twist.linear.x, wp.twist.twist.linear.y, wp.twist.twist.linear.z]
            wp_ang   = [wp.twist.twist.angular.x, wp.twist.twist.angular.y, wp.twist.twist.angular.z]
            
            base_pos.append(wp_pos)
            base_posXY.append(wp_posXY)
            base_ori.append(wp_ori)
            base_vel.append(wp_vel)
            base_ang.append(wp_ang)
            
        self.base_pos   = np.array(base_pos)
        self.base_posXY = np.array(base_posXY)
        self.base_ori   = np.array(base_ori)
        self.base_vel   = np.array(base_vel)
        self.base_ang   = np.array(base_ang)
        
        # store base_waypoints        
        self.waypoints = msg

    def traffic_cb(self, msg):
        # store lights data in numpy arrays
        lights_posXY = []     # Position, but only XY:     x, y
        lights_state = []     # state [Green=2, Yellow=1, Red=0]
        for light in msg.lights:
            light_posXY = [light.pose.pose.position.x, light.pose.pose.position.y]
            light_state = light.state
            
            lights_posXY.append(light_posXY)
            lights_state.append(light_state)
            
        self.lights_posXY = np.array(lights_posXY)
        self.lights_state = np.array(lights_state)
        
        # store lights from styx_msgs as well        
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """         
        
        
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            if light_wp != NO_LIGHT:
                #light_wp = light_wp if state == TrafficLight.RED else -1
                if state != TrafficLight.RED:
                    light_wp = -light_wp # >0: RED; <0: NOT RED!
            
            self.last_wp    = light_wp
            if DEBUG:
                rospy.logwarn("tl_detector: publishing new  light_wp = %s +++++++++++++++++++++++",str(light_wp))            
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            if DEBUG:
                rospy.logwarn("tl_detector: publishing last light_wp = %s ***********************",str(self.last_wp))            
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        
        # only increase count if we are looking at a light
        if light_wp != NO_LIGHT:
            self.state_count += 1

        
    def get_closest_waypoint(self, position):
        """Identifies the closest path waypoint in front of the car to the given position
           https://en.wikipedia.org/wiki/Closest_pair_of_points_problem (NOT USED...)
        
        Args:
            position ([x,y]): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """    
        closest = None
        
        #
        # Logic:
        # - Find the closest waypoint to position p1
        # - Next waypoint p2 from list of waypoints
        # - Calculate dot product of these vectors:
        #     v1: p1->p2
        #     v2: p1->position
        # - If dot product is positive then p2 is next waypoint else p1
        
        if self.waypoints: 
            pos  = np.array( [position] ) # 2D array, valid for cdist
            dist = distance.cdist(self.base_posXY, pos)
            i1   = np.argmin(dist)  
            
            if i1 == len(self.base_posXY)-1:
                i2 = 0
            else:
                i2 = i1+1      
            pos = pos.flatten() # get rid of 2nd dimension
            
            p1 = self.base_posXY[i1]
            p2 = self.base_posXY[i2]    
            v1 = p2  - p1
            v2 = pos - p1
            dot_product = np.dot(v1,v2)
            
            if dot_product < 0:
                closest = i1
            else:
                closest = i2            
        
        return closest

    def get_closest_light(self):
        """Identifies the closest light in front of the current car location
        
        Returns:
            closest_light: index of closest light in front of the car
        """
        #TODO implement
        
        closest_light = None
        
        if self.waypoints and self.pose and len(self.lights)>0:
            car_posXY = [self.pose.pose.position.x, self.pose.pose.position.y]
            wp_i1     = self.get_closest_waypoint(car_posXY) # index of closest waypoint in front of car
            # pick one further out, to get more stable dot product calculation
            if wp_i1 == len(self.base_posXY)-1:
                wp_posXY = self.base_posXY[0]
            else:
                wp_posXY = self.base_posXY[wp_i1+1] 
            
            car_posXY       = np.array( [ car_posXY ] ) # 2D array, valid for cdist
            dist            = distance.cdist(self.lights_posXY, car_posXY)
            closest_light   = np.argmin(dist) 
            car_posXY       = car_posXY.flatten() # get rid of 2nd dimension
            
        
            # check if closest one is behind the car. If it is, then we need to pick the next light.
            tl_posXY = self.lights_posXY[closest_light]    
            v1 = wp_posXY  - car_posXY
            v2 = tl_posXY  - car_posXY
            dot_product = np.dot(v1,v2)
            
            if dot_product > 0: 
                pass # found closest light in front of car
            else:
                # light is behind car, pick next one
                if closest_light == len(self.lights)-1:
                    closest_light = 0
                else:
                    closest_light += 1
            
        return closest_light  # return the index of the closest light in front of the car

    def get_light_state_async(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        self.async_light_state = self.light_classifier.get_classification(cv_image)

        self.isClassifierAvailable = True
    
    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if( self.isClassifierAvailable ):
            self.isClassifierAvailable = False
            thread = threading.Thread(target=self.get_light_state_async, args=[])
            thread.start()

        return self.async_light_state
            

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose.pose)

            #TODO find the closest visible traffic light (if one exists)
            #if car_position:
            closest_light = self.get_closest_light()
            if closest_light is not None:
                light = self.lights[closest_light]
                # get waypoint of stopping line in front of light
                stopline_wp = self.get_closest_waypoint(stop_line_positions[closest_light])
                
                #if DEBUG:
                #    light_wp = self.get_closest_waypoint([self.lights[closest_light].pose.pose.position.x,
                #                                                      self.lights[closest_light].pose.pose.position.y])
                #
                #    car_wp   = self.get_closest_waypoint([self.pose.pose.position.x,
                #                                                      self.pose.pose.position.y])                    
                #    
                #    rospy.logwarn("tl_detector: stopline_wp, light_wp, car_wp= %s, %s, %s",
                #                  str(stopline_wp), str(light_wp), str(car_wp))
                
                if USE_SIMULATOR_STATE:
                    state = self.lights_state[closest_light]
                    return stopline_wp, state
                else:
                    state = self.get_light_state(light)
                    return stopline_wp, state
        
        self.waypoints = None
        return NO_LIGHT, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
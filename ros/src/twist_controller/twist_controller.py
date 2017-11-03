import rospy
from pid import PID
from yaw_controller import YawController

DEBUG = False              # get printout

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, 
                       max_lat_accel, max_steer_angle):
        # TODO: Implement
        
        # controller for throttle & brake        
        #self.tc = PID(0.1, 0.002, 0.0, mn=-0.5, mx=1.0) # kp, ki, kd as taken from Term 1 pid project
        self.tc = PID(0.1, 0.002, 0.0, mx=1.0) # kp, ki, kd as taken from Term 1 pid project
        
        # controller for steer
        self.yc = YawController(wheel_base, steer_ratio, min_speed, 
                                max_lat_accel, 
                                max_steer_angle)
        
        # just for testing
        self.test_0_status = 0
        self.test_0_speed  = 5

    def control(self, 
                target_linear_velocity, target_angular_velocity,
                current_linear_velocity, current_angular_velocity,
                sample_time):

        throttle = 0.0
        brake    = 0.0
        steer    = 0.0
        
        error = target_linear_velocity - current_linear_velocity
        if error < 0.0:
            # Need to brake
            # TODO: Calculate this from target acceleration, in unit of torque (N*m)
            brake = -1.0E30
        else:    
            if sample_time > 0.0:
                thc   = self.tc.step(error, sample_time)
                if thc > 0:
                    throttle = thc
        
        if DEBUG:
            rospy.logwarn("twist_controller: target_v, current_v, throttle, brake, steer = %e, %e %e, %e, %e",
                          target_linear_velocity, current_linear_velocity, throttle, brake, steer)
               
            
        steer = self.yc.get_steering(target_linear_velocity,
                                     target_angular_velocity,
                                     current_linear_velocity)
        
        return throttle, brake, steer

    def control_test(self, 
                target_linear_velocity, target_angular_velocity,
                current_linear_velocity, current_angular_velocity,
                sample_time,
                test=0):
        """
        Function to test speeding up & braking
        """
        
        throttle = 0.0
        brake    = 0.0
        steer    = 0.0

        if test == 0:
            # Speed up until certain speed is reached
            # Then brake until stopped
            
            if self.test_0_status == 0:
                # speed up till speed reached
                if current_linear_velocity <= self.test_0_speed:
                    throttle = 1.0
                else:
                    self.test_0_status = 1
                    
            if self.test_0_status == 1:
                # break till stop
                if current_linear_velocity > 1.E-6:  # never really reaches 0.0...??...
                    brake = -1.E15
                else:
                    self.test_0_status = 0   # speed back up             

            if DEBUG:
                rospy.logwarn("twist_controller speed, throttle,brake= %e, %f, %f",current_linear_velocity,throttle,brake)
    
        steer = self.yc.get_steering(target_linear_velocity,
                                     target_angular_velocity,
                                     current_linear_velocity)

        return throttle, brake, steer
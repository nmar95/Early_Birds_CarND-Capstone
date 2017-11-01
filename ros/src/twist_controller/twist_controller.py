from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, 
                       max_lat_accel, max_steer_angle):
        # TODO: Implement
        
        # controller for throttle & brake        
        self.tc = PID(0.1, 0.002, 0.0, mn=-0.5, mx=1.0) # kp, ki, kd as taken from Term 1 pid project
        
        # controller for steer
        self.yc = YawController(wheel_base, steer_ratio, min_speed, 
                                max_lat_accel, 
                                max_steer_angle)
        pass

    def control(self, 
                target_linear_velocity, target_angular_velocity,
                current_linear_velocity, current_angular_velocity,
                sample_time):

        throttle = 0.0
        brake    = 0.0
        steer    = 0.0
        
        if sample_time > 0.0:
            error = target_linear_velocity - current_linear_velocity
            thc   = self.tc.step(error, sample_time)
            if thc > 0:
                throttle = thc
            else:
                brake = thc
            
            steer = self.yc.get_steering(target_linear_velocity,
                                         target_angular_velocity,
                                         current_linear_velocity)
        
        return throttle, brake, steer

from pid import PID
from yaw_controller import YawController
import math
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']

        min_speed = 0
        
        self.velocity_pid = PID(kp=0.8, ki=0, kd=0.05, mn=self.decel_limit, mx=0.5 * self.accel_limit)
        self.steering_pid = PID(kp=0.16, ki=0.001, kd=0.1, mn=-self.max_steer_angle, mx=self.max_steer_angle)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle)
        

    def reset(self):
        self.velocity_pid.reset()
        self.steering_pid.reset()

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity, cross_track_error, sample_time):
        linear_velocity_error = target_linear_velocity - current_linear_velocity

        velocity_correction = self.velocity_pid.step(linear_velocity_error, sample_time)

        brake = 0
        throttle = velocity_correction

        if(throttle < 0):
            deceleration = abs(throttle)
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * deceleration if deceleration > self.brake_deadband else 0.
            throttle = 0

        predictive_steering = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)
        # without Steering PID car will wiggle around the waypoint path
        corrective_steering = self.steering_pid.step(cross_track_error, sample_time)
        steering = predictive_steering + corrective_steering
        return throttle, brake, steering

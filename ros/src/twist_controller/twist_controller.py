import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH= 0.44704
MIN_SPEED = 0.1  # m/s

# The PID values are taken from the walk-through video.
# They explain the values were determined experimentally.
# some_dude/mine/course
# PID_KP = 0.3#0.2    #0.3  # proportional term for PID
# PID_KI = 0.003#0.0004 #0.1  # integral term for PID
# PID_KD = 4.0#3.0    #0.0  # derivative term for PID

PID_KP = 1.0
PID_KI = 0.1
PID_KD = 10.0

MIN_THROTTLE = 0.0
MAX_THROTTLE = 0.2
LOW_PASS_TAU = 0.5
LOW_PASS_TS = 0.02

# Value tuned for Carla, as explained in the walk-through.
TORQUE_TO_KEEP_VEHICLE_STATIONARY = 400  # Nm


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(
            PID_KP, PID_KI, PID_KD, MIN_THROTTLE, MAX_THROTTLE)
        self.v_low_pass_filter = LowPassFilter(LOW_PASS_TAU, LOW_PASS_TS)
        self.last_time = rospy.get_time()


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0, 0, 0
        
        rospy.logwarn("Target angular velocity: %f" % angular_vel)
        rospy.logwarn("Target linear velocity: %f" % linear_vel)
        rospy.logwarn("Current linear velocity: %f" % current_vel)
        
        current_time = rospy.get_time()
        current_vel = self.v_low_pass_filter.filt(current_vel)
        rospy.logwarn("Filtered linear velocity: %f" % current_vel)
        vel_error = linear_vel - current_vel

        steering = self.yaw_controller.get_steering(
            linear_vel, angular_vel, current_vel)
        throttle = self.throttle_controller.step(
            error=vel_error,
            sample_time=current_time - self.last_time)
        brake = 0

        if linear_vel == 0 and current_vel < 0.1:
            # The vehicle is stopped at a traffic light.
            throttle = 0
            brake = TORQUE_TO_KEEP_VEHICLE_STATIONARY
        elif throttle < 0.1 and vel_error < 0:
            # Velocity error is negative, so we need to slow down.
            throttle = 0
            assert self.decel_limit < 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
            
        rospy.logwarn("Throttle: %f" % throttle)
        rospy.logwarn("Brake: %f" % brake)
        rospy.logwarn("Steering: %f" % steering)
        
        # Update object state.
        self.last_vel = current_vel
        self.last_time = rospy.get_time()

        return throttle, brake, steering
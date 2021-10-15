
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, \
        accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        kp = 0.3 
        ki = 0.1
        kd = 0.
        mn = 0. # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp,ki,kd,mn,mx)

        # filter high frequency noise
        tau = 0.5 # 1/(2pi*tau) = cut of frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau,ts)



        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, target_vel, target_angel):
        # Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # Filter
        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Current Angular vel: {0}" .format(current_angel))
        # rospy.logwarn("Target Angular vel: {0}" .format(target_angel))
        # rospy.logwarn("Current velocity: {0}" .format(current_vel))
        # rospy.logwarn("Filtered velocity: {0}" .format(self.vel_lpf.get()))
        # rospy.logwarn("Target velocity: {0}" .format(target_vel))

        # TIME
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Brake and Throttle -------------------------------------
        # accelerate 
        vel_error = target_vel - current_vel
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.

        if target_vel == 0. and current_vel < 0.1:
            # Standstill
            throttle = 0.
            brake = 700 #N*m - to hold the car in place if we are stopped at a light. Accelevaration ~ 1m/s^2
        elif throttle <0.1 and vel_error < 0.:
            # Braking
            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        # Steering ------------------------------------------------
        # positive - left
        # negative - right
        steering = self.yaw_controller.get_steering(target_vel, target_angel, current_vel)

        #rospy.logwarn("Steering: {0}" .format(steering))
        #rospy.logwarn("Throttle: {0}" .format(throttle))
        #rospy.logwarn("Brake: {0}" .format(brake))

        return throttle, brake, steering

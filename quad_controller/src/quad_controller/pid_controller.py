# Copyright (C) 2017 Udacity Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        #TODO
        
        # The PID controller can be initialised with a specific kp
        # ki, and kd
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)

        # Set max wind up
        self.max_windup_ = float(max_windup)

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.error_sum_ = 0.0
        self.last_error_ = 0.0

    def reset(self):
        #TODO
        self.set_point_ = 0.0
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0
        self.error_sum_ = 0.0
        self.last_timestamp_ = 0.0
        self.last_error_ = 0

    def setTarget(self, target):
        #TODO
        self.set_point_ = float(target)

    def setKP(self, kp):
        #TODO
        self.kp_ = float(kp)

    def setKI(self, ki):
        #TODO
        self.ki_ = float(ki)

    def setKD(self, kd):
        #TODO
        self.kd_ = float(kd)

    def setMaxWindup(self, max_windup):
        #TODO
        self.max_windup_ = int(max_windup)

    def update(self, measured_value, timestamp):
        #TODO
        
        # Calculate the time step
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
        	return 0

        # Calculate the error
        error = self.set_point_ - measured_value

        # Set the last time stamp
        self.last_timestamp_ = timestamp

        # Sum the errors
        self.error_sum_ += error * delta_time

        # Find the delta error
        delta_error = error - self.last_error_

        # Update the past error
        self.last_error_ = error

        # Address max windup
        if self.error_sum_ > self.max_windup_:
        	self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.error_sum_:
        	self.error_sum_ = -self.max_windup_

        # Proportional error
        p = self.kp_ * error

        # Integral error
        i = self.ki_ * self.error_sum_

        # Derivative effort
        d = self.kd_ * delta_error / delta_time

        # Set the control effort
        u = p + i + d

        return u
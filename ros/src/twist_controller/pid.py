import rospy

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):
        assert sample_time >= 0
        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        #rospy.logwarn("(Throttle PID) Error: %f" % error)
        #rospy.logwarn("(Throttle PID) Sample time: %f" % sample_time)
        #rospy.logwarn("(Throttle PID) Integral: %f" % integral)
        #rospy.logwarn("(Throttle PID) Derivative: %f" % derivative)
        
        val = self.kp * error + self.ki * integral + self.kd * derivative;
        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
            
        self.last_error = error
        return val

#!/usr/bin/env python
# Copyright (c) 2022 Lucas Walter
# BSD-3 license
# output an integrated value given an input velocity

import copy
from threading import Lock

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Float32


class Integrator(object):
    def __init__(self):
        self.t0 = rospy.Time.now()
        self.timer = None

        self.position = 0.0
        # TODO(lucasw) handle a number of output topics, including marti stamped Float32?
        self.pub = rospy.Publisher("position", Float32, queue_size=4)

        self.lock = Lock()
        self.config = None
        self.ddr = DDynamicReconfigure("")
        # The dynamic reconfigure probably won't go lower than 0.1,
        # 0.01 only works for topics
        self.ddr.add_variable("enable", "enable", True)
        self.ddr.add_variable("period", "update period", 0.025, 0.01, 10.0)
        self.ddr.add_variable("use_expected_dt", "use expected time stamps instead of actual", True)
        self.ddr.add_variable("reset", "reset to start_pos", False)
        self.ddr.add_variable("start_pos", "starting value", 0.0, -5.0, 5.0)
        max_vel = rospy.get_param("~max_vel", 1.0)
        self.ddr.add_variable("velocity", "velocity", 0.0, -max_vel, max_vel)
        self.ddr.start(self.config_callback)

    def is_changed(self, old_config, config, param):
        if old_config is None:
            return True

        return config[param] != old_config[param]

    def config_callback(self, config, level):
        with self.lock:
            old_config = copy.deepcopy(self.config)

        if self.is_changed(old_config, config, 'period'):
            if self.timer:
                self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration(config.period), self.update)

        if config.reset or old_config is None:
            self.position = config.start_pos
            config.reset = False

        with self.lock:
            self.config = copy.deepcopy(config)
        return config

    def update(self, event):
        with self.lock:
            config = copy.deepcopy(self.config)
        if config is None:
            return
        if event.last_real is None or event.last_real == rospy.Time(0):
            return

        if config.use_expected_dt:
            dt = (event.current_expected - event.last_expected).to_sec()
        else:
            dt = (event.current_real - event.last_real).to_sec()

        # TODO(lucasw) keep track of old velocity for a triangle here
        self.position += config.velocity * dt
        self.pub.publish(Float32(self.position))


if __name__ == '__main__':
    rospy.init_node('integrator')
    integrator = Integrator()
    rospy.spin()

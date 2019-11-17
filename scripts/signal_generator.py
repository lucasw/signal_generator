#!/usr/bin/env python
# Copyright (c) 2019 Lucas Walter
# BSD-3 license
# November 2019

import math
import rospy

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Float32


class SignalGenerator(object):
    def __init__(self):
        self.t0 = rospy.Time.now()
        self.timer = 0
        self.config = None

        self.pub = rospy.Publisher("signal", Float32, queue_size=2)

        self.ddr = DDynamicReconfigure("")
        self.ddr.add_variable("dr_server", "dynamic reconfigure server", "")
        self.ddr.add_variable("name", "dr parameter name", "")
        self.ddr.add_variable("period", "update period", 0.1, 0.01, 10.0)
        self.num_freqs = 4
        for i in range(self.num_freqs):
            si = str(i)
            fr = float(i) / float(self.num_freqs)
            self.ddr.add_variable("f" + si, "frequency " + si, 1.0 - fr * fr, 0.0, 10.0)
            self.ddr.add_variable("a" + si, "amplitude " + si,
                                  fr, 0.0, 2.0)
            self.ddr.add_variable("p" + si, "phase " + si, 0.0, -math.pi, math.pi)
            # TODO(lucasw) sine, sawtooth, ramp, square
        self.ddr.start(self.config_callback)

    def is_changed(self, config, param):
        if self.config is None:
            return True

        return config[param] != self.config[param]

    def config_callback(self, config, level):
        if self.is_changed(config, 'period'):
            self.timer = rospy.Timer(rospy.Duration(config.period), self.update)

        self.config = config
        return config

    def update(self, event):
        if self.config is None:
            return

        dt = (event.current_real - self.t0).to_sec()
        val = 0.0
        for i in range(self.num_freqs):
            si = str(i)
            # TODO(lucasw) want to smoothly transition when
            # frequencies change, to do that need to offset phase in a way
            # that puts the old frequency and the new one at the same value.
            theta = 2.0 * math.pi * self.config["f" + si] * dt + self.config["p" + si]
            val += self.config["a" + si] * math.sin(theta)

        self.pub.publish(Float32(val))

if __name__ == '__main__':
    rospy.init_node('signal_generator')
    signal_generator = SignalGenerator()
    rospy.spin()

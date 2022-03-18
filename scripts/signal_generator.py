#!/usr/bin/env python
# Copyright (c) 2019 Lucas Walter
# BSD-3 license
# November 2019

import math
import rospy
import threading

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from dynamic_reconfigure.client import Client
from std_msgs.msg import Float32


class SignalGenerator(object):
    def __init__(self):
        self.t0 = rospy.Time.now()
        self.timer = None
        self.config = None
        self.dr_client = None
        self.new_server = False
        self.pub = None

        self.ddr = DDynamicReconfigure("")
        self.ddr.add_variable("server", "dynamic reconfigure server", "")
        self.ddr.add_variable("param", "dr parameter name", "")
        self.ddr.add_variable("topic", "topic name", "signal")
        # The dynamic reconfigure probably won't go lower than 0.1,
        # 0.01 only works for topics
        self.ddr.add_variable("period", "update period", 0.05, 0.01, 10.0)
        self.num_freqs = 3
        for i in range(self.num_freqs):
            si = str(i)
            fr = float(i) / float(self.num_freqs)
            self.ddr.add_variable("freq" + si, "frequency " + si, 1.0 - fr * fr, 0.0, 10.0)
            self.ddr.add_variable("amp" + si, "amplitude " + si,
                                  fr, 0.0, 2.0)
            self.ddr.add_variable("phase" + si, "phase " + si, 0.0, -math.pi, math.pi)
            self.ddr.add_variable("offset" + si, "offset " + si, 0.0, -5.0, 5.0)
            # TODO(lucasw) sine, sawtooth, ramp, square
        self.ddr.start(self.config_callback)

    def is_changed(self, config, param):
        if self.config is None:
            return True

        return config[param] != self.config[param]

    def config_callback(self, config, level):
        if self.is_changed(config, 'period'):
            if self.timer:
                self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration(config.period), self.update)
        if self.is_changed(config, 'server'):
            self.new_server = True
        if self.is_changed(config, 'topic'):
            self.pub = rospy.Publisher(config.topic, Float32, queue_size=2)

        self.config = config
        return config

    def server_dr_callback(self, config):
        rospy.logdebug(config)

    def safe_update_config(self, values):
        if self.dr_client is None:
            return False

        update_timeout = 1.0
        # TODO(lucasw) could follow
        # https://stackoverflow.com/questions/2829329/catch-a-threads-exception-in-the-caller-thread-in-python
        # and pass a message back if the update configuration fails
        try:
            th1 = threading.Thread(target=self.dr_client.update_configuration,
                                   args=[values])
            th1.start()
            t1 = rospy.Time.now()
            while ((rospy.Time.now() - t1).to_sec() < update_timeout):
                if th1.isAlive():
                    rospy.sleep(0.05)
                else:
                    break
            if th1.isAlive():
                # TODO(lucasw) how to kill t1- or does it matter?
                raise RuntimeError("timeout")
        except RuntimeError as ex:
            # self.dr_client = None
            rospy.logerr(f"{ex} {self.config.server} {self.config.param}")
            return False
        return True

    def connect_server(self):
        if self.config.server == '':
            self.dr_client = None
            self.new_server = False
            return True
        try:
            self.dr_client = Client(self.config.server,
                                    timeout=0.05,
                                    config_callback=self.server_dr_callback)
            self.new_server = False
            rospy.loginfo("connected to new server '" + self.config.server + "'")
        except Exception as ex:
            rospy.logdebug_throttle(5.0, ex)
            rospy.logerr_throttle(5.0, "no server available '" + self.config.server + "'")
            return False
        return True

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
            theta = 2.0 * math.pi * self.config["freq" + si] * dt + self.config["phase" + si]
            val += self.config["amp" + si] * math.sin(theta) + self.config["offset" + si]

        if self.pub is not None:
            self.pub.publish(Float32(val))

        if self.new_server:
            self.connect_server()

        self.safe_update_config({self.config.param: val})


if __name__ == '__main__':
    rospy.init_node('signal_generator')
    signal_generator = SignalGenerator()
    rospy.spin()

#!/usr/bin/env python
#  Copyright (c) 2019 Connor Yates, Scott Chow, Christopher Bollinger, Christopher Eriksen
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.

import rospy
from std_msgs.msg import String, Bool
from std_msgs.msg import Time


class Agreer:
    def __init__(self):
        rospy.Subscriber("time_coord", Time, self.check_time)
        rospy.Subscriber("switch_found", Bool, self.ready_cb)
        self.comm_pub = rospy.Publisher("accept_feedback", String, queue_size=10)
        self.ready = False

    def ready_cb(self, msg):
        if msg.data:
            self.ready = True

    def check_time(self, msg):
        rospy.logdebug("Time received")
        print msg.data.secs
        if rospy.get_time() + 6 < msg.data.secs < rospy.get_time() + 12:
            self.comm_pub.publish("Accept")
            rospy.logdebug("Sleeping {}".format(msg.data.secs - rospy.get_time()))

            rospy.logdebug("Future time should be: {}".format(msg.data.secs))
            rospy.sleep(msg.data.secs - rospy.get_time())
            rospy.logdebug("Current time is: {}".format(rospy.get_time()))
            rospy.logwarn("Executing maneuver")
            self.ready = False
        else:
            self.comm_pub.publish("Reject")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # don't start again until reset
            while self.ready and not rospy.is_shutdown():
                rate.sleep()
            while not self.ready and not rospy.is_shutdown():
                rate.sleep()

            self.comm_pub.publish("In Position")


if __name__ == '__main__':
    rospy.init_node("agreer", log_level=rospy.DEBUG)
    agreer = Agreer()
    agreer.run()

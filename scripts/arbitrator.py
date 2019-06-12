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
from std_msgs.msg import String, Bool, Time


class Arbitrator:
    def __init__(self):
        self.comm_pub = rospy.Publisher("time_coord", Time, queue_size=10)
        rospy.Subscriber("accept_feedback", String, self.feedback_log)
        rospy.Subscriber("switch_found", Bool, self.self_ready_cb)
        self.state = None
        self.other_accept = False
        self.received = False
        self.other_in_pos = False
        self.in_pos = False

    def self_ready_cb(self, msg):
        self.in_pos = msg.data

    def feedback_log(self, msg):
        rospy.loginfo("Arbitrator received {}".format(msg.data))
        if msg.data == "In Position":
            self.other_in_pos = True
        if msg.data == "Accept":
            self.other_accept = True
            self.received = True
        if msg.data == "Reject":
            self.received = True

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.in_position and self.other_in_pos:
                break
            rate.sleep()

        while not rospy.is_shutdown():
            future_time = int(rospy.get_time() + 10)
            msg = Time()
            msg.data.secs = future_time
            self.comm_pub.publish(msg)
            while not self.received:
                rate.sleep()
            if self.other_accept:
                break
            else:
                self.received = False

        # RUN THE MANEUVER NOW
        rospy.logdebug("Sleeping {}".format(future_time - rospy.get_time()))
        rospy.logdebug("Future time should be: {}".format(future_time))
        rospy.sleep(future_time - rospy.get_time())
        rospy.logdebug("Current time is: {}".format(rospy.get_time()))
        rospy.logwarn("Executing maneuver")


if __name__ == '__main__':
    rospy.init_node("arbitrator")
    arbitrator = Arbitrator()
    arbitrator.run()

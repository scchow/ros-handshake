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
from std_msgs.msg import String
from std_msgs.msg import Time


class Arbitrator(object):
    def __init__(self):
        self.comm_pub = rospy.Publisher("time_coord", Time)
        rospy.Subscriber("accept_feedback", String, self.feedback_log)
        self.state = None
        self.other_accept = False
        self.received = False

    def feedback_log(self, msg):
        if msg.data == "In Position":
            self.run()
        if msg.data == "Accept":
            self.other_accept = True
            self.received = True
        if msg.data == "Reject":
            self.received = True

    def run(self):
        rate = rospy.Rate(0.1)
        while True:
            future_time = rospy.get_time() + 5
            self.comm_pub.publish(Time(future_time))
            while not self.received:
                rate.sleep()
            if self.other_accept:
                break
            else:
                self.received = False
        # RUN THE MANEUVER NOW


if __name__ == '__main__':
    rospy.init_node("arbitrator")
    arbitrator = Arbitrator()
    rospy.spin()

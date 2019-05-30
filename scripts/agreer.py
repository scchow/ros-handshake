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


class Agreer(object):
    def __init__(self):
        rospy.Subscriber("time_coord", Time, self.check_time)
        self.comm_pub = rospy.Publisher("accept_feedback", String)
        self.comm_pub.publish("In Position")  # One-off for this first test

    def check_time(self, msg):
        if rospy.Time(3) < msg.data < rospy.Time(6):
            self.comm_pub.publish("Accept")
        else:
            self.comm_pub.publish("Reject")


if __name__ == '__main__':
    rospy.init_node("agreer")
    agreer = Agreer()
    rospy.spin()

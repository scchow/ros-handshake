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
from std_msgs.msg import String
from std_msgs.msg import Time


class Explorer:
    def __init__(self):
        self.rooms = rosparam.get_param('~rooms')
        self.robot = rosparam.get_param('~robot_num')
        # takes the form
        # ~/rooms/A/x
        # ~/rooms/A/y
        # ~/rooms/B/x
        # ~/rooms/B/y
        # ...
        self.explore_pub = rospy.Publisher('explore_coordination', Time, queue_size=10)
        rospy.Subscriber('explore_coordination', String, self.explore_cb)

        self.switch_present = [-1 for r in rooms.keys()]
        self.state = None
        self.other_accept = False
        self.received = False
        self.other_in_pos = False

    def explore_cb(self, msg):
        rospy.logdebug('Explore message: {}'.format(msg.data))
        # message format {robot#}:{E/T/F}:{A/B/C/D}
        if msg.data[0] == self.robot:
            pass  # ignore messages we send
        else:
            if msg.data[2] == 'E':
                pass
            elif msg.data[2] == 'T':
                pass
            elif msg.data[2] == 'F':
                pass
            elif msg.data[2] == 'T':

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for k in self.explored:
                if not self.explored[k]:
                    

if __name__ == '__main__':
    rospy.init_node("arbitrator")
    arbitrator = Arbitrator()
    arbitrator.run()

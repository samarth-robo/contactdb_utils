#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf2_msgs.msg import TFMessage

class TfRemapper:
    def __init__(self):
        self.pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        mappings = rospy.get_param('~mappings', [])
        self.mappings = {}

        for i in mappings:
            if "old" in i and "new" in i:
                self.mappings[i["old"]] = i["new"]

        print("Applying the following mappings to incoming tf frame ids",
        self.mappings)
        rospy.Subscriber("/tf_old", TFMessage, self.callback)

    def callback(self, tf_msg):
        for transform in tf_msg.transforms:
            if transform.header.frame_id in self.mappings:
                transform.header.frame_id = self.mappings[transform.header.frame_id]
            if transform.child_frame_id  in self.mappings:
                transform.child_frame_id = self.mappings[transform.child_frame_id]

        self.pub.publish(tf_msg)


if __name__ == '__main__':
    rospy.init_node('tf_remapper')
    tfr = TfRemapper()
    rospy.spin()
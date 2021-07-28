import sys
import argparse

# OpenCV
import cv2

# Ros libraries
import rospy

# Ros Messages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


VERBOSE = False


class ListenImages:
    def __init__(self, topic):
        '''Initialize ros publisher, ros subscriber'''
        self.topic = topic
        self.subscriber = rospy.Subscriber(self.topic,
                                           Image, self.callback, queue_size=1)

    def callback(self, ros_data):
        '''Callback function of subscribed topic.'''

        # cvt to np
        bridge = CvBridge()
        image_np = bridge.imgmsg_to_cv2(ros_data, desired_encoding='passthrough')

        # show images
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)


def main(args):
    '''Initializes and cleanup ros node'''
    li = ListenImages(args.topic)
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print
        "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # rosbag info *.bag - info about the topics
    parser = argparse.ArgumentParser(description="Listen")
    parser.add_argument("topic", help="topic to subscrive")
    args = parser.parse_args()

    main(args)

import sys

import os
import cv2
import rospy
import rosbag
import progressbar
from datetime import datetime
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
from cv_bridge import CvBridge
import argparse
import numpy as np
import datetime as dt


class KittiInfo():
    def __init__(self, base_path, sequence, **kwargs):
        self.sequence_path = os.path.join(base_path, 'sequences', sequence)
        self.frames = kwargs.get('frames', None)

        self._load_timestamps()

    def _load_timestamps(self):
        """Load timestamps from file."""
        timestamp_file = os.path.join(self.sequence_path, 'times.txt')

        # Read and parse the timestamps
        self.timestamps = []
        with open(timestamp_file, 'r') as f:
            for line in f.readlines():
                t = dt.timedelta(seconds=float(line))
                self.timestamps.append(t)

        # Subselect the chosen range of frames, if any
        if self.frames is not None:
            self.timestamps = [self.timestamps[i] for i in self.frames]

def read_calib_file(filepath):
    """Read in a calibration file and parse into a dictionary."""
    data = {}

    with open(filepath, 'r') as f:
        for line in f.readlines():
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass

    return data

def save_camera_data(bag, kitti_info, util, bridge, camera, camera_frame_id, topic, initial_time):
    print("Exporting camera {}".format(camera))

    camera_pad = '{0:01d}'.format(camera)
    image_path = os.path.join(kitti_info.sequence_path, 'image_{}'.format(camera_pad))
    image_filenames = sorted(os.listdir(image_path))
    image_datetimes = map(lambda x: initial_time + x.total_seconds(), kitti_info.timestamps)

    calib = CameraInfo()
    calib.header.frame_id = camera_frame_id
    calib.P = util['P{}'.format(camera_pad)]

    iterable = zip(image_datetimes, image_filenames)
    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)
        calib.height, calib.width = cv_image.shape[:2]
        if camera in (0, 1):
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        encoding = "mono8"
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = camera_frame_id

        image_message.header.stamp = rospy.Time.from_sec(dt)
        topic_ext = "/cam" + str(camera)

        calib.header.stamp = image_message.header.stamp
        bag.write(topic + topic_ext, image_message, t=image_message.header.stamp)
        bag.write(topic + '/camera_info', calib, t=calib.header.stamp)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Convert KITTI dataset to ROS bag file the easy way!")
    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))
    parser.add_argument("dir", nargs="?", default=os.getcwd(),
                        help="base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-t", "--date",
                        help="date of the raw dataset (i.e. 2011_09_26), option is only for RAW datasets.")
    parser.add_argument("-r", "--drive",
                        help="drive number of the raw dataset (i.e. 0001), option is only for RAW datasets.")
    parser.add_argument("-s", "--sequence", choices=odometry_sequences,
                        help="sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.")
    args = parser.parse_args()

    bridge = CvBridge()
    compression = rosbag.Compression.NONE
    cameras = [
        (0, 'camera_gray_left', '/kitti/camera_gray_left'),
        (1, 'camera_gray_right', '/kitti/camera_gray_right'),
    ]

    bag = rosbag.Bag("kitti_data_odometry_sequence_{}.bag".format(args.sequence), 'w',
                     compression=compression)

    # kitti = pykitti.odometry(args.dir, args.sequence)
    # kitti_sequence_path = os.path.join(args.dir, 'sequences', args.sequence)
    kitti_info = KittiInfo(args.dir, args.sequence)
    if not os.path.exists(kitti_info.sequence_path):
        print('Path {} does not exists. Exiting.'.format(kitti_info.sequence_path))
        sys.exit(1)

    try:
        util = read_calib_file(os.path.join(args.dir, 'sequences', args.sequence, 'calib.txt'))
        current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
        # Export
        used_cameras = cameras[:2]

        for camera in used_cameras:
            save_camera_data(bag, kitti_info, util, bridge, camera=camera[0], camera_frame_id=camera[1],
                             topic=camera[2], initial_time=current_epoch)

    finally:
        print("## OVERVIEW ##")
        print(bag)
        bag.close()

#!/usr/bin/python
from __future__ import print_function, division
import yaml
import sys
import os
import rospy
from vio_ros.msg import VioSensorMsg
from std_msgs.msg import String
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


def vio_sensor_cb(data):
    global cnt, active, imgs
    num_samples = 100  # number of image samples to take

    if cnt == num_samples and active:
        imgs['l'] /= num_samples
        imgs['r'] /= num_samples
        active = 0
        return

    left = CvBridge().imgmsg_to_cv2(data.left_image, "mono8")
    left = np.float32(left)#/256.0
    right = CvBridge().imgmsg_to_cv2(data.right_image, "mono8")
    right = np.float32(right)#/256.0

    if cnt == 0:
        imgs['l'] = left
        imgs['r'] = right
    else:
        cv2.accumulate(left, imgs['l'])
        cv2.accumulate(right, imgs['r'])

    cnt += 1


def device_serial_nr_cb(data):
    global device_serial_nr
    device_serial_nr = data.data


if __name__ == "__main__":
    rospy.init_node('calibrate_dark_current')

    device_serial_nr = None
    cnt = 0
    active = 1
    imgs = {'l': [], 'r': []}

    print('To calibrate the dark current, make sure the lenses are completely covered.')
    raw_input('Press enter to start calibration...')

    rospy.Subscriber("/vio_sensor", VioSensorMsg, vio_sensor_cb, queue_size=1)
    rospy.Subscriber("/vio_sensor/device_serial_nr", String, device_serial_nr_cb, queue_size=1)

    rate = rospy.Rate(10)
    while active and not rospy.is_shutdown():
        rate.sleep()

    if not rospy.is_shutdown():
        alpha = 0.2  # why is this necessary? I don't know but it makes the result better
        left = cv2.convertScaleAbs(imgs['l'], alpha=alpha)
        right = cv2.convertScaleAbs(imgs['r'], alpha=alpha)

        duo_path = rospkg.RosPack().get_path('duo3d_ros')

        # select lens
        lenses = os.listdir(os.path.join(duo_path, 'calib', device_serial_nr))
        lenses = [lens for lens in lenses if os.path.isdir(os.path.join(duo_path, 'calib', device_serial_nr, lens))]

        if len(lenses) == 1:
            print('Found one lens: {}. Using that one.'.format(lenses[0]))
            lens = lenses[0]
        else:
            print('Found several lenses:')
            for i, lens in enumerate(lenses):
                print('{}: {}'.format(i+1, lens))
            selection = int(raw_input('Select the lens you want by providing the appropriate number: '))
            if selection < 1 or selection > len(lenses):
                raise Exception('The provided number {} is not in the valid range [{}:{}]'.format(selection, 1, len(lenses)))
            lens = lenses[selection-1]

        resolution = '{}x{}'.format(left.shape[0], left.shape[1])

        cv2.imwrite(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution, 'dark_current_l.bmp'), left)
        cv2.imwrite(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution, 'dark_current_r.bmp'), right)
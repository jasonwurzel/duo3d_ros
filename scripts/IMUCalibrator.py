#!/usr/bin/python
from __future__ import print_function, division
import yaml
import sys
import os
import rospy
from vio_ros.msg import VioSensorMsg
from std_msgs.msg import String
import rospkg


def vio_sensor_cb(data):
    global gyr, acc, cnt, active, user_ready

    if not user_ready:
        return
    num_samples = 1000  # number of IMU samples to capture

    for imu in data.imu:
        gyr['x'].append(imu.angular_velocity.x)
        gyr['y'].append(imu.angular_velocity.y)
        gyr['z'].append(imu.angular_velocity.z)

        acc['x'].append(imu.linear_acceleration.x)
        acc['y'].append(imu.linear_acceleration.y + 9.81)
        acc['z'].append(imu.linear_acceleration.z)

        cnt += 1

    if cnt >= num_samples:
        gyr['x'] = sum(gyr['x'])/len(gyr['x'])
        gyr['y'] = sum(gyr['y'])/len(gyr['y'])
        gyr['z'] = sum(gyr['z'])/len(gyr['z'])

        acc['x'] = sum(acc['x'])/len(acc['x'])
        acc['y'] = sum(acc['y'])/len(acc['y'])
        acc['z'] = sum(acc['z'])/len(acc['z'])
        print('\nFinished capturing data.')
        active = 0

    if cnt >= num_samples:
        cnt = 0
        user_ready = 0


def device_serial_nr_cb(data):
    global device_serial_nr
    device_serial_nr = data.data


if __name__ == "__main__":
    rospy.init_node('calibrate_IMU')

    gyr = {'x': [], 'y': [], 'z': []}
    acc = {'x': [], 'y': [], 'z': []}

    cnt = 0
    active = 1
    user_ready = 0
    device_serial_nr = None

    rospy.Subscriber("/vio_sensor", VioSensorMsg, vio_sensor_cb, queue_size=1)
    rospy.Subscriber("/vio_sensor/device_serial_nr", String, device_serial_nr_cb, queue_size=1)

    print('Waiting for device serial nr...')
    while not device_serial_nr:
        rospy.sleep(0.5)
    print('... done')

    print('Calibrating IMU. Place the device with zero roll and pitch (i.e. facing horizontally forward). Do not move the device')
    rate = rospy.Rate(100)
    while active and not rospy.is_shutdown():
        if not user_ready:
            raw_input("Press Enter when ready...")
            user_ready = 1
            print('Recording data...')
        rate.sleep()
    
    if not rospy.is_shutdown():

        rospack = rospkg.RosPack()
        duo_path = rospack.get_path('duo3d_ros')

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

        # select resolution
        resolutions = os.listdir(os.path.join(duo_path, 'calib', device_serial_nr, lens))
        resolutions = [resolution for resolution in resolutions if os.path.isdir(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution))]

        if len(resolutions) == 1:
            print('Found one resolution: {}. Using that one.'.format(resolutions[0]))
            resolution = resolutions[0]
        else:
            print('Found several resolutions:')
            for i, resolution in enumerate(resolutions):
                print('{}: {}'.format(i+1, resolution))
            selection = int(raw_input('Select the resolution you want by providing the appropriate number: '))
            if selection < 1 or selection > len(resolutions):
                raise Exception('The provided number {} is not in the valid range [{}:{}]'.format(selection, 1, len(resolutions)))
            resolution = resolutions[selection-1]

        # load the yaml file
        with open(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution, 'cameraParams.yaml'), 'r') as infile:
            cameraParams = yaml.load(infile)

        print('For each axis of the accelerometer and gyroscope you can decide to use the new estimate (answer y), keep the old one (answer n).')

        print('Accelerometer biases:')
        print('Old       \tNew')
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['acc_bias'][0][0], acc['x']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['acc_bias'][1][0], acc['y']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['acc_bias'][2][0], acc['z']))

        axes = 'xyz'
        for i in range(3):
            try:
                selection = raw_input('Do you want to use the new accelerometer {} axis estimate? [Y/n]: '.format(axes[i]))
            except EOFError:
                print('')
                sys.exit(-1)

            if not selection or selection == 'y' or selection == 'Y':
                cameraParams['acc_bias'][i][0] = acc[axes[i]]

        print('Gyroscope biases:')
        print('Old       \tNew')
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][0][0], gyr['x']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][1][0], gyr['y']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][2][0], gyr['z']))

        for i in range(3):
            try:
                selection = raw_input('Do you want to use the new gyroscope {} axis estimate? [Y/n]: '.format(axes[i]))
            except EOFError:
                print('')
                sys.exit(-1)

            if not selection or selection == 'y' or selection == 'Y':
                cameraParams['gyro_bias'][i][0] = gyr[axes[i]]

        with open(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution, 'cameraParams.yaml'), 'w') as outfile:
            outfile.write(yaml.dump(cameraParams, default_flow_style=None))

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
    global active_axis, gyr, acc, cnt, active, user_ready

    if not user_ready:
        return
    num_samples = 500  # number of IMU samples per axis

    cnt += 1
    if active_axis == 0:  # gyro
        for imu in data.imu:
            gyr['x'].append(imu.angular_velocity.x)
            gyr['y'].append(imu.angular_velocity.y)
            gyr['z'].append(imu.angular_velocity.z)

        if cnt == num_samples:
            gyr['x'] = sum(gyr['x'])/len(gyr['x'])
            gyr['y'] = sum(gyr['y'])/len(gyr['y'])
            gyr['z'] = sum(gyr['z'])/len(gyr['z'])
            print('done. {} {} {}'.format(gyr['x'], gyr['y'], gyr['z']))
            print('\nCalibrating positive X axis. Place the device such that this axis points down.')
    elif active_axis == 1:  # acc_x+
        for imu in data.imu:
            acc['xp'].append(imu.linear_acceleration.x + 9.81)

        if cnt == num_samples:
            acc['xp'] = sum(acc['xp'])/len(acc['xp'])
            print('done. {}'.format(acc['xp']))
            print('\nCalibrating negative X axis. Place the device such that this axis points down.')
    elif active_axis == 2:  # acc_x-
        for imu in data.imu:
            acc['xn'].append(imu.linear_acceleration.x - 9.81)

        if cnt == num_samples:
            acc['xn'] = sum(acc['xn'])/len(acc['xn'])
            print('done. {}'.format(acc['xn']))
            print('\nCalibrating positive Y axis. Place the device such that this axis points down.')
    elif active_axis == 3:  # acc_y+
        for imu in data.imu:
            acc['yp'].append(imu.linear_acceleration.y + 9.81)

        if cnt == num_samples:
            acc['yp'] = sum(acc['yp'])/len(acc['yp'])
            print('done. {}'.format(acc['yp']))
            print('\nCalibrating negative Y axis. Place the device such that this axis points down.')
    elif active_axis == 4:  # acc_y-
        for imu in data.imu:
            acc['yn'].append(imu.linear_acceleration.y - 9.81)

        if cnt == num_samples:
            acc['yn'] = sum(acc['yn'])/len(acc['yn'])
            print('done. {}'.format(acc['yn']))
            print('\nCalibrating positive Z axis. Place the device such that this axis points down.')
    elif active_axis == 5:  # acc_z+
        for imu in data.imu:
            acc['zp'].append(imu.linear_acceleration.z + 9.81)

        if cnt == num_samples:
            acc['zp'] = sum(acc['zp'])/len(acc['zp'])
            print('done. {}'.format(acc['zp']))
            print('\nCalibrating negative Z axis. Place the device such that this axis points down.')
    elif active_axis == 6:  # acc_z-
        for imu in data.imu:
            acc['zn'].append(imu.linear_acceleration.z - 9.81)

        if cnt == num_samples:
            acc['zn'] = sum(acc['zn'])/len(acc['zn'])
            print('done. {}'.format(acc['zn']))

            print('\nFinished capturing data.')
            active = 0

    if cnt == num_samples:
        active_axis += 1
        cnt = 0
        user_ready = 0


def device_serial_nr_cb(data):
    global device_serial_nr
    device_serial_nr = data.data


if __name__ == "__main__":
    rospy.init_node('calibrate_IMU')

    gyr = {'x': [], 'y': [], 'z': []}
    acc = {'xp': [], 'xn': [], 'yp': [], 'yn': [], 'zp': [], 'zn': []}

    cnt = 0
    active_axis = 0  # gyro, acc_x, acc_y, acc_z
    active = 1
    user_ready = 0
    device_serial_nr = None

    rospy.Subscriber("/vio_sensor", VioSensorMsg, vio_sensor_cb, queue_size=1)
    rospy.Subscriber("/vio_sensor/device_serial_nr", String, device_serial_nr_cb, queue_size=1)

    print('Calibrating gyroscope. Do not move the device')
    rate = rospy.Rate(100)
    while active and not rospy.is_shutdown():
        if not user_ready:
            raw_input("Press Enter when ready...")
            user_ready = 1
            print('Recording data...')
        rate.sleep()
    
    if not rospy.is_shutdown():
        acc['x'] = (acc['xp'] + acc['xn'])/2
        acc['y'] = (acc['yp'] + acc['yn'])/2
        acc['z'] = (acc['zp'] + acc['zn'])/2

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
            selection = raw_input('Do you want to use the new accelerometer {} axis estimate? [Y/n]: '.format(axes[i]))
            if not selection or selection == 'y' or selection == 'Y':
                cameraParams['acc_bias'][i][0] = acc[axes[i]]

        print('Gyroscope biases:')
        print('Old       \tNew')
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][0][0], gyr['x']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][1][0], gyr['y']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][2][0], gyr['z']))

        for i in range(3):
            selection = raw_input('Do you want to use the new gyroscope {} axis estimate? [Y/n/m]: '.format(axes[i]))
            if not selection or selection == 'y' or selection == 'Y':
                cameraParams['gyro_bias'][i][0] = gyr[axes[i]]

        with open(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution, 'cameraParams.yaml'), 'w') as outfile:
            outfile.write(yaml.dump(cameraParams, default_flow_style=None))

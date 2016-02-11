#!/usr/bin/env python
import rospy
from vio_ros.msg import VioSensorMsg
import cv2
import cv_bridge
import dynamic_reconfigure.client


def vio_sensor_cb(data):
    global target_brightness, dynamic_reconfigure_client, dynamic_reconfigure_config, recompute_delay, recompute_cnt, controller_gain, average_lowpass, filtered_mean, exposure_change

    recompute_cnt += 1
    if not recompute_cnt % recompute_delay:
        mean_l = cv2.mean(cv_bridge.CvBridge().imgmsg_to_cv2(data.left_image, desired_encoding="passthrough"))
        mean_r = cv2.mean(cv_bridge.CvBridge().imgmsg_to_cv2(data.right_image, desired_encoding="passthrough"))
        mean_brightness = (mean_l[0] + mean_r[0])/2

        filtered_mean = (1-average_lowpass) * filtered_mean + average_lowpass * mean_brightness

        exposure_change = -controller_gain * (filtered_mean - target_brightness)

    exposure_change_actual = max(-max_step_size, min(max_step_size, exposure_change))

    new_exposure = dynamic_reconfigure_config['exposure'] + exposure_change_actual

    exposure_change -= exposure_change_actual

    # print('exposure_change {} exposure_change_actual {}'.format(exposure_change, exposure_change_actual))

    params = {'exposure': new_exposure}
    dynamic_reconfigure_config = dynamic_reconfigure_client.update_configuration(params)


if __name__ == "__main__":

    rospy.init_node('duo_auto_exposure')

    target_brightness = rospy.get_param('~target_brightness', 256 / 2)  # desired average brightness
    recompute_frequency = rospy.get_param('~recompute_frequency', 1)  # how many times per second to recompute the mean
    controller_gain = rospy.get_param('~controller_gain', 1)  # gain that translates mean brightness error to change in exposure
    average_lowpass = rospy.get_param('~average_lowpass', 0.5)  # gain of lowpass filter that filters the mean brightness measurement
    average_lowpass = max(0, min(1, average_lowpass))
    max_step_size = rospy.get_param('~max_step_size', 1)  # maximum exposure change per frame
    camera_fps = rospy.get_param('/duo_node/FPS', 1)
    recompute_delay = camera_fps / recompute_frequency
    recompute_cnt = 0
    filtered_mean = target_brightness
    exposure_change = 0

    dynamic_reconfigure_client = dynamic_reconfigure.client.Client('duo_node')
    dynamic_reconfigure_config = dynamic_reconfigure_client.update_configuration({})

    rospy.Subscriber("/vio_sensor", VioSensorMsg, vio_sensor_cb, queue_size=1)

    rospy.spin()
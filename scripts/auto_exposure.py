#!/usr/bin/env python
import rospy
from vio_ros.msg import VioSensorMsg
import cv2
import cv_bridge
import dynamic_reconfigure.client


def vio_sensor_cb(data):
    global target_brightness, dynamic_reconfigure_client, dynamic_reconfigure_config, recompute_delay, recompute_cnt, controller_gain

    recompute_cnt += 1
    if recompute_cnt % recompute_delay:
        # print('skipping')
        return

    mean_l = cv2.mean(cv_bridge.CvBridge().imgmsg_to_cv2(data.left_image, desired_encoding="passthrough"))
    mean_r = cv2.mean(cv_bridge.CvBridge().imgmsg_to_cv2(data.right_image, desired_encoding="passthrough"))
    mean_brightness = (mean_l[0] + mean_r[0])/2
    
    exposure_change = -controller_gain * (mean_brightness - target_brightness)

    new_exposure = dynamic_reconfigure_config['exposure'] + exposure_change

    params = {'exposure': new_exposure}
    dynamic_reconfigure_config = dynamic_reconfigure_client.update_configuration(params)


if __name__ == "__main__":

    rospy.init_node('duo_auto_exposure')

    target_brightness = rospy.get_param('~target_brightness', 256 / 2)
    recompute_frequency = rospy.get_param('~recompute_frequency', 1)
    controller_gain = rospy.get_param('~controller_gain', 1)

    camera_fps = rospy.get_param('/duo_node/FPS', 1)
    recompute_delay = camera_fps / recompute_frequency
    recompute_cnt = 0

    dynamic_reconfigure_client = dynamic_reconfigure.client.Client('duo_node')
    dynamic_reconfigure_config = dynamic_reconfigure_client.update_configuration({})

    rospy.Subscriber("/vio_sensor", VioSensorMsg, vio_sensor_cb, queue_size=1)

    rospy.spin()
#! /usr/bin/env python

import time
import rospy
import cv2
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def gstreamer_pipeline(
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

if __name__=="__main__":
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        
    rospy.init_node('fw_camera_node')
    bridge = CvBridge()
    pub = rospy.Publisher('camera', Image, queue_size=1)
    rate = rospy.Rate(10)
    
    try:
        while not rospy.is_shutdown():
            ref,frame = cap.read()
            if not ref:
                rospy.loginfo("Not Found Devices")
                break
            image_msg = bridge.cv2_to_imgmsg(frame,"bgr8")
            pub.publish(image_msg)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Exiting Program")
    
    except Exception as exception_error:
        rospy.loginfo("Error occurred. Exiting Program")
        rospy.loginfo("Error: " + str(exception_error))
    
    finally:
        cap.release()
        pass


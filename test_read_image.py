#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import matplotlib.pyplot as plt

bridge = CvBridge()

from datetime import datetime

last = 0

def callback(msg: Image):
    global last

    now = datetime.now().timestamp()

    if now - last > 1:
        # only update once per second
        last = now
    else:
        return

    # print(msg.height, msg.width)
    print("Image saved at", now)
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    plt.imshow(image)
    plt.savefig(f"test.png")

def main():
    rospy.init_node("camera_subscriber")
    rospy.Subscriber("ur5e/camera1/image_raw", Image, callback)
    
    # rospy.spin()
    import time ; time.sleep(0.2)


if __name__ == "__main__":
    main()

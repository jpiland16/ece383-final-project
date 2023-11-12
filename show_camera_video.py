#!/usr/bin/python3

"""
This code shows the live video from a ROS/gazebo camera.
10 November 2023
"""

import rospy
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()


blank_image = np.zeros([800, 800, 3], dtype=np.uint8)
current_image = blank_image
last_callback_str = "Waiting for image..."

def callback(msg: Image):
    global current_image, last_callback_str
    current_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    last_callback_str = datetime.now().isoformat()

def create_matplotlib_animation():
    """
    Create a matplotlib animation that loops repeatedly, only tracing the
    position of the end effector in the XY plane. (deprecated)
    """

    fig, ax = plt.subplots()
    im = plt.imshow(blank_image)
    title = ax.text(400, 400, last_callback_str)

    def init():
        return im, title

    def update(frame):
        im.set_data(current_image)
        title.set_text(last_callback_str)
        return im, title

    ani = FuncAnimation(fig, update, frames=None,
                        init_func=init, blit=True)


def main():
    rospy.init_node("live_graph_listener")
    rospy.Subscriber("ur5e/camera1/image_raw", Image, callback)

    create_matplotlib_animation()
    plt.show()

if __name__ == "__main__":
    main()


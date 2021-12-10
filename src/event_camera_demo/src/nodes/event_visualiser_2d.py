import rospy
from event_camera_demo.msg import EventPacket
from sensor_msgs.msg import Image
import numpy as np
import ros_numpy as rnp
import threading
from datetime import datetime
import threading

topic_out = None

def events_to_image(data):
    # get event data
    x = data.x
    y = data.y
    p = np.array(data.polarity)

    # setup display image
    im_out = np.zeros((data.height, data.width, 3), dtype=np.uint8)

    # set events to blue/red for positive/negative events
    im_out[y, x, p * 2] = 255
    
    # send message
    msg = rnp.msgify(Image, im_out, encoding='rgb8')
    topic_out.publish(msg)


def start_node(blocking=True):
    global topic_out
    topic_out = rospy.Publisher('/event_camera_demo/events_demo/event_visual_2d', Image, queue_size=1)
    rospy.init_node('event_to_visualiser_2d', anonymous=False)
    rospy.Subscriber("/davis346/data/event_packets", EventPacket, events_to_image)
    if blocking:
        rospy.spin()

    
if __name__ == '__main__':
    start_node()

import rospy
from event_camera_demo.msg import EventPacket
from sensor_msgs.msg import Image
import numpy as np
import ros_numpy as rnp
from cv_bridge import CvBridge
from pprint import pprint
import sys
# this node subscribes to the davis346 node and creates an event visual from the event data

bridge = CvBridge()
topic_out = None

def events_to_image(data):
    # get events from packet (convert ROS image to numpy matrix)
    events = bridge.imgmsg_to_cv2(data.events, desired_encoding='64FC1').astype(np.uint64)

    # get event data
    x = events[:, 1]
    y = events[:, 2]
    p = np.array(events[:, 3])

    # setup display image
    im_out = np.zeros((data.height, data.width, 3), dtype=np.uint8)

    # set events to blue/red for positive/negative events
    im_out[y, x, p * 2] = 255
    # im_out[y, x, :] = 255
    
    # send message
    msg = rnp.msgify(Image, im_out, encoding='rgb8')
    topic_out.publish(msg)


def start_node(blocking=True):
    # output topic for 2d visualisation
    global topic_out
    topic_out = rospy.Publisher('/event_camera_demo/event_visual_2d', Image, queue_size=1)
    # node
    rospy.init_node('event_to_visualiser_2d', anonymous=False)
    # input topic for packets of events
    rospy.Subscriber("/event_camera_demo/event_packets", EventPacket, events_to_image)

    # allows this node to be started from an external script more easily
    if blocking:
        rospy.spin()

    
if __name__ == '__main__':
    start_node(True)

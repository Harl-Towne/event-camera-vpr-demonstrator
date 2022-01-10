import rospy
from demo_msgs.msg import EventPacket
from cv_bridge import CvBridge
import numpy as np
from pprint import pprint
import tonic

pub = None
printed = False
bridge = CvBridge()
denoise_transform = tonic.transforms.Denoise(filter_time=1000000)

# datatpe used to store events
event_dtype = np.dtype([("t", np.int64), ('x', np.int64), ('y', np.int64), ('p', np.int64)])

def callback(data_in):
    global printed, denoise_transform

    # convert ROS image containing events to numpy matrix
    event_packet = bridge.imgmsg_to_cv2(data_in.events, desired_encoding='64FC1')

    # create empty numpy array for events to be stored into (this array matches datatype used to store events)
    events = np.empty(event_packet.shape[0], dtype=event_dtype)

    # transfer events from numpy matrix to numpy array, dtype conversions are implicit (see event_dtype)
    for i, column in enumerate(['t', 'x', 'y', 'p']):
        if column == 't':
            events[column] = event_packet[:, i] * 10e6
        else:
            events[column] = event_packet[:, i]

    events_denoised = denoise_transform(events)

    data_out = EventPacket()
    data_out.width = data_in.width
    data_out.height = data_in.height

    events_out = np.empty((len(events_denoised), 4), dtype=np.float64)
    for i, column in enumerate(['t', 'x', 'y', 'p']):
        if column == 't':
            events_out[:, i] = events_denoised[column] / 10e6
        else:
            events_out[:, i] = events_denoised[column]

    data_out.events = bridge.cv2_to_imgmsg(events_out, encoding="64FC1")

    pub.publish(data_out)

def start_node(subsribe_to="event_camera_demo/event_packets", publish_to="event_camera_demo/event_display", block=True):
    global pub
    rospy.init_node("null_filter", anonymous=True)
    rospy.Subscriber(subsribe_to, EventPacket, callback)
    pub = rospy.Publisher(publish_to, EventPacket, queue_size=10)
    if block:
        rospy.spin()


if __name__ == '__main__':
    start_node()
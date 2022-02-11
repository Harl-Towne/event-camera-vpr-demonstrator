import rospy
from qcr_event_msgs.msg import EventPacket

# this node does not affect or change the data, it simply passed the data through
# its purpose is to fill the slot where a filter node should be if a filter is not wanted

pub = None

def callback(data):
    pub.publish(data)

def start_node(subsribe_to="event_camera_demo/event_packets", publish_to="event_camera_demo/event_display", block=True):
    global pub
    rospy.init_node("null_filter", anonymous=True)
    rospy.Subscriber(subsribe_to, EventPacket, callback)
    pub = rospy.Publisher(publish_to, EventPacket, queue_size=10)
    if block:
        rospy.spin()


if __name__ == '__main__':
    start_node()
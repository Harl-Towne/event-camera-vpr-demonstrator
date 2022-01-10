import rospy
from demo_msgs.msg import EventPacket

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
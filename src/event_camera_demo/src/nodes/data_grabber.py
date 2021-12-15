import rospy
from sensor_msgs.msg import Image


def start_node(topic, msg_type, callback, blocking=True):
    # node
    rospy.init_node('data_grabber', anonymous=True)
    # input topic for packets of events
    rospy.Subscriber(topic, msg_type, callback)

    # allows this node to be started from an external script more easily
    if blocking:
        rospy.spin()

    
if __name__ == '__main__':
    print("auto starting grabber on topic: /event_camera_demo/events_demo/event_visual_2d")
    start_node('/event_camera_demo/events_demo/event_visual_2d', Image, lambda data: print("Got data:", type(data)), True)

import rospy
from dvs_msgs.msg import EventArray
from sensor_msgs.msg import Image
import numpy as np
import ros_numpy as rnp

topic_out = None

def events_to_image(data):
    im_out = np.zeros((data.height, data.width, 3), dtype=np.uint8)

    data_len = min(len(data.events), 10000)
    events = data.events[:data_len-1]

    x = np.zeros(data_len, dtype=np.uint)
    y = np.zeros(data_len, dtype=np.uint)
    s = np.zeros(data_len, dtype=np.uint)    
    for i, event in enumerate(events):
        x[i] = event.x
        y[i] = event.y
        s[i] = event.polarity

    im_out[y, x, s * 2] = 255
    msg = rnp.msgify(Image, im_out, encoding='rgb8')
    topic_out.publish(msg)  


def start_node(blocking=True):
    global topic_out
    topic_out = rospy.Publisher('/event_camera_demo/events_demo/event_visual_2d', Image, queue_size=1)
    rospy.init_node('event_to_visualiser_2d', anonymous=False)
    rospy.Subscriber("dvs/events", EventArray, events_to_image)
    if blocking:
        rospy.spin()

    
if __name__ == '__main__':
    start_node()

from dv import NetworkNumpyEventPacketInput
import numpy as np
import rospy
from event_camera_demo.msg import EventPacket
from cv_bridge import CvBridge
import traceback

# This node was made for use with dv-gui/dv-runtime software
# it is now obselete with the rpg_dvs_ros driver

bridge = CvBridge()

def start_node(blocking=True):
    # output topic for events
    events_topic = rospy.Publisher('/event_camera_demo/event_packets', EventPacket, queue_size=1)
    rospy.init_node('davis346', anonymous=False)

    rospy.loginfo("Node 'davis346' Started")

    # allows this node to be started from an external script more easily
    if blocking:
        publish_data(events_topic)
    else:
        pass
        #TODO: start node in non-blocking way (maybe is best just to do this from the main ui?)

def publish_data(events_topic):
    while not rospy.is_shutdown():  # loop until shutdown
        try:
            with NetworkNumpyEventPacketInput(address='127.0.0.1', port=7777) as event_packets: # connect to dv software over tcp
                rospy.loginfo("Connected to davis346 camera")
                while not rospy.is_shutdown(): # loop untill shutdown or error
                    # get next packet of events
                    events = event_packets.__next__()

                    # get position, polarity and timestamp from packet
                    x = events['x']
                    y = events['y']
                    p = events['polarity']
                    ts = events['timestamp']
                    
                    # setup message
                    # event data is sent as a ros image because it is much faster than sending it through arrays (something to to with ROS serialisation or something, idk)
                    msg = EventPacket()
                    msg.height = 260
                    msg.width = 346
                    data = np.array([ts, x, y, p], dtype=np.float64).transpose()
                    msg.events = bridge.cv2_to_imgmsg(data, encoding="64FC1") 

                    # send message
                    events_topic.publish(msg)

        except Exception as e: # usualy connection errors
            rospy.logerr(traceback.format_exc())
            rospy.sleep(1)
            rospy.loginfo("trying to reconnect")  

    
if __name__ == '__main__':
    try:
        start_node(True)
    except rospy.ROSInterruptException:
        pass

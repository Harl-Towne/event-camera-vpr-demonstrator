from dv import NetworkNumpyEventPacketInput
from datetime import datetime
import numpy as np
import ros_numpy as rnp
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import rospy
from pprint import pprint
from event_camera_demo.msg import EventPacket
from datetime import datetime
from cv_bridge import CvBridge



def start_node():
    events_topic = rospy.Publisher('/davis346/data/event_packets', EventPacket, queue_size=1)
    events_topic2 = rospy.Publisher('/davis346/data/event_packets2', Image, queue_size=1)
    publishtime_topic = rospy.Publisher('/davis346/data/publish_time', Int32, queue_size=10)
    rospy.init_node('davis346', anonymous=False)

    rospy.loginfo("Node 'davis346' Started")

    while not rospy.is_shutdown():  # loop until shutdown
        try:
            with NetworkNumpyEventPacketInput(address='127.0.0.1', port=7777) as event_packets: # connect to dv software over tcp
                rospy.loginfo("Connected to davis346 camera")
                while not rospy.is_shutdown(): # loop untill shutdown or error
                    print("#"*20)
                    ss = datetime.now()
                    s = datetime.now()
                    # get next packet of events
                    events = event_packets.__next__()

                    print("get packet:\t\t", datetime.now()-s)
                    s = datetime.now()

                    # get position, polarity and timestamp from packet
                    x = events['x']
                    y = events['y']
                    p = events['polarity']
                    ts = events['timestamp']

                    x2 = np.array(events['x'], dtype=np.uint64)
                    y2 = np.array(events['y'], dtype=np.uint64)
                    p2 = np.array(events['polarity'], dtype=np.uint64)
                    ts2 = np.array(events['timestamp'], dtype=np.uint64)
                    
                    # d = datetime.now()-s
                    print("extracting data:\t", datetime.now()-s)
                    s = datetime.now()
                    
                    # data_len = len(x)#min(5000, len(x))
                    # setup message
                    msg = EventPacket()
                    msg.height = 260
                    msg.width = 346
                    msg.x = x#[:data_len]
                    msg.y = y#[:data_len]
                    msg.polarity = p#[:data_len]
                    msg.timestamp = ts#[:data_len]

                    print("creating message 1:\t", datetime.now()-s)
                    s = datetime.now()

                    data = np.array([x, y, p, ts]).transpose()
                    # msg2 = bridge.cv2_to_imgmsg(data, encoding="passthrough")

                    print("creating message 2:\t", datetime.now()-s)
                    s = datetime.now()

                    # sent message
                    events_topic.publish(msg)

                    d = datetime.now()-s
                    print("sending message 1:\t", datetime.now()-s)
                    s = datetime.now()

                    # events_topic2.publish(msg2)

                    # d = datetime.now()-s
                    print("sending message 2:\t", datetime.now()-s)
                    s = datetime.now()
                    print("total:\t\t\t", datetime.now()-ss)

                    msg2 = Int32()
                    msg2.data = d.microseconds
                    print(msg2.data)
                    publishtime_topic.publish(msg2)

        except Exception as e: # usualy connection errors
            rospy.logerr("Error: {}".format(e))
            rospy.sleep(1)
            rospy.loginfo("trying to reconnect")  

    
if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass

from PyQt5 import QtWidgets, uic, QtCore, QtGui
import sys
import os

from numpy.core.records import record
from cv_bridge import CvBridge
import rospy
from pprint import pprint
from demo_msgs.msg import EventPacket
import numpy as np
from enum import Enum

bridge = CvBridge()

# live - showing live events from camera
# record - showing live events from camera but also saving them for later
# playback - showing previously recorded events
# pause - showing previously recorded events but not moving in time
states = Enum("states", ("live", "record", "playback", "pause"))

# main class handles pretty well everything
class EventDemoWindow(QtWidgets.QMainWindow):

    fps = 30 # frames to display each second
    min_frame_size = 0.40 # minimum frame width (measurd in equalivalent min playback speed. basicaly when the playback speed is lowwer than this number the frames stop getting smaller and start overlapping so that there are still enought events each frame to be able to see something)

    # datatpe used to store events
    event_dtype = np.dtype([("timestamp", np.float64), ('x', np.uint16), ('y', np.uint16), ('polarity', np.uint8)])

    def __init__(self):
        super().__init__()

        # initial state is live display
        self.state = states.live

        # some class variables that shouldn't be modified (because the class will do that itself)
        self.last_frame_end=None # used to keep track of where the recodring is up to
        self.image_width = None  # width and height of camera display
        self.image_height = None
        self.recorded_data = np.empty(0, dtype=self.event_dtype)

        # load ui
        uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "video_demo.ui"), self)

        # setup ui events
        self.recordBtn.clicked.connect(self.recordBtn_clicked)
        self.playBtn.clicked.connect(self.playBtn_clicked)
        self.stopBtn.clicked.connect(self.stopBtn_clicked)
        self.playspeedBar.valueChanged.connect(self.speedUpdate_bar)
        self.playspeedBox.valueChanged.connect(self.speedUpdate_box)

        # display refresh timer
        self.updateTimer = QtCore.QTimer()
        self.updateTimer.timeout.connect(self.display_new_playback_frame)
        self.updateTimer.start(1000//self.fps)

        # setup node
        rospy.init_node('event_demo_ui', anonymous=True)
        rospy.Subscriber("event_camera_demo/event_packets", EventPacket, self.new_event_packet)

    # fucntion handles the record button being pushed
    def recordBtn_clicked(self):
        if self.state == states.live:
            self.move_to_state(states.record)
        else: # only other possible state should be states.record
            self.move_to_state(states.playback)

    # fucntion handles the play button being pushed
    def playBtn_clicked(self):
        if self.state == states.playback:
            self.move_to_state(states.pause)
        else: # only other possible state should be states.pause
            self.move_to_state(states.playback)

    # fucntion handles the stop button being pushed
    def stopBtn_clicked(self):
        self.move_to_state(states.live)

    # function handles the tranistion between states
    # some buttons are only active in certain states
    # some buttons have different function is different states (their text changes)
    def move_to_state(self, new_state):
        # save new state
        self.state = new_state

        if new_state == states.live:
            self.recordBtn.setEnabled(True)
            self.recordBtn.setText("Record")
            self.playBtn.setText("Play")
            self.stopBtn.setEnabled(False)

        elif new_state == states.record:
            self.recorded_data = np.empty(0, dtype=self.event_dtype) # erase old recording to make way for new one
            self.recordBtn.setEnabled(True)
            self.recordBtn.setText("Stop Recording")
            self.playBtn.setEnabled(False)
            self.stopBtn.setEnabled(False)

        elif new_state == states.playback:
            self.recordBtn.setEnabled(False)
            self.playBtn.setEnabled(True)
            self.playBtn.setText("Pause")
            self.stopBtn.setEnabled(True)

        elif new_state == states.pause:
            self.recordBtn.setEnabled(False)
            self.playBtn.setEnabled(True)
            self.playBtn.setText("Resume")
            self.stopBtn.setEnabled(True)

        else: # this should never happen but just in case
            self.move_to_state(states.live)

    # function for syncronising play speed bar and play speed box
    # updates box based on bar
    def speedUpdate_bar(self, value):
        self.playspeedBox.setValue(value/100)

    # function for syncronising play speed bar and play speed box
    # updates bar based on box
    def speedUpdate_box(self, value):
        self.playspeedBar.setValue((int)(value*100))

    # callback function for ROS subscription
    # extracts events and saves them based on state
    def new_event_packet(self, data):

        # convert ROS image containing events to numpy matrix
        event_packet = bridge.imgmsg_to_cv2(data.events, desired_encoding='64FC1')

        # create empty numpy array for events to be stored into (this array matches datatype used to store events)
        events = np.empty(event_packet.shape[0], dtype=self.event_dtype)

        # transfer events from numpy matrix to numpy array, dtype conversions are implicit (see event_dtype)
        for i, column in enumerate(['timestamp', 'x', 'y', 'polarity']):
            events[column] = event_packet[:, i]

        # if the data is currently being recorded also save it into the recorded_data array
        if self.state == states.record:
            if len(self.recorded_data) == 0:
                self.recorded_data = events
            else:            
                self.recorded_data = np.append(self.recorded_data, events)


        # this if statement triggers once for the first event packet and signals to rest of the class to start generating frames
        # also saves image width and height (which should never change)
        if self.last_frame_end == None:
            self.last_frame_end = events[-1]['timestamp']
            self.image_width = data.width
            self.image_height = data.height

            
        if self.state == states.live or  self.state == states.record:
            self.display_new_frame(events)

    # calcualtes the new time range for the next frame to be displayed on screen
    # self.state == states.pause will cause this function to return the same time range (assuming playback_speed doesn't change)
    def new_frame_window(self, playback_speed):
        frame_step = 1/(self.fps/playback_speed)
        frame_width = max(frame_step, 1/(self.fps/self.min_frame_size))
        # only advance frame if the data is being played back (not paused)
        if self.state == states.playback:
            # move end for frame by one frame (adjusting for playback speed)
            self.last_frame_end += frame_step
            # if the end of the frame is over the end of the data
            if self.last_frame_end > self.recorded_data[-1]['timestamp'] or self.last_frame_end < self.recorded_data[0]['timestamp'] + frame_width:
                # set the end of the frame back the the start of the data (adjusting for frame width)
                self.last_frame_end = self.recorded_data[0]['timestamp'] + frame_width

        return self.last_frame_end - frame_width, self.last_frame_end

    # cut one frame worth of event data out of the recorded event data and then display it
    def display_new_playback_frame(self):
        # abort function if no data has been received yet
        if self.last_frame_end == None:
            return

        # save state just in case it changes while this functon is running
        # it shouldn't, but just in case
        state = self.state

        playback_speed = 1 # default speed
        
        if state == states.playback or state == states.pause:
            # if recorded data is being shown get the recorded data and playback speed
            data = self.recorded_data
            playback_speed = self.playspeedBox.value()
        else:
            # don't display new data if live data is being displayed
            return

        # abort if no data
        # this happens with the live playback sometimes
        # not sure why
        if len(data) == 0:
            return

        # get timestamps and convert frame range in time to frame range in index
        timestamps = data['timestamp']
        time_start, time_end = self.new_frame_window(playback_speed)
        index_start = np.searchsorted(timestamps, time_start)
        index_end = np.searchsorted(timestamps, time_end, side='right')

        # get just the data from this frame
        data = data[index_start:index_end+1]

        self.display_new_frame(data)

    # display the given event data as a frame
    def display_new_frame(self, event_data):
        # get stuff
        x = event_data['x']
        y = event_data['y']
        p = event_data['polarity']

        # setup display image
        frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # use stuff to make image
        frame[y, x, p * 2] = 255
        # don't ask about the , self.image_width*3 i don't know what it does but it's important
        image = QtGui.QImage(frame.copy(), self.image_width, self.image_height, self.image_width*3, QtGui.QImage.Format_RGB888) # conver np image to qt image
        pixmap = QtGui.QPixmap(image) # convert qt image to qt pixmap
        pixmap = pixmap.scaled(self.imageDisplay.width(),self.imageDisplay.height(), QtCore.Qt.KeepAspectRatio) # scale pixmap
        self.imageDisplay.setPixmap(pixmap) # display pixmap



if __name__ == '__main__':
    # start GUI
    app = QtWidgets.QApplication(sys.argv)
    window = EventDemoWindow()
    window.show()
    sys.exit(app.exec())
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
from datetime import datetime

bridge = CvBridge()

# live - showing live events from camera
# record - showing live events from camera but also saving them for later
# playback - showing previously recorded events
# pause - showing previously recorded events but not moving in time
states = Enum("states", ("live", "record", "playback", "pause", 'back'))

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
        self.last_state = states.live

        # some class variables that shouldn't be modified (because the class will do that itself)
        self.last_frame_end=None # used to keep track of where the recodring is up to
        self.image_width = None  # width and height of camera display
        self.image_height = None
        self.recorded_data = np.empty(0, dtype=self.event_dtype)
        self.user_integration_interval = None
        self.last_playback_indices = [0, 20000]

        # load ui
        uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "video_demo.ui"), self)

        # setup ui events
        self.playbackBtn.clicked.connect(self.playbackBtn_clicked)
        self.recordBtn.clicked.connect(self.recordBtn_clicked)
        self.playspeedBar.valueChanged.connect(self.speedUpdate_bar)
        self.playspeedBox.valueChanged.connect(self.speedUpdate_box)
        self.integrationBar.valueChanged.connect(self.integrationUpdate_bar)
        self.integrationBox.valueChanged.connect(self.integrationUpdate_box)
        self.integrationCheck.stateChanged.connect(self.integrationUpdate_check)
        self.playtimeBar.sliderPressed.connect(self.playtimeBar_pressed)
        self.playtimeBar.sliderReleased.connect(self.playtimeBar_released)
        self.playtimeBar.valueChanged.connect(self.playtimeBar_dragged)

        # display refresh timer
        self.updateTimer = QtCore.QTimer()
        self.updateTimer.timeout.connect(self.display_new_playback_frame)
        self.updateTimer.start(1000//self.fps)

        # setup node
        rospy.init_node('event_demo_ui', anonymous=True)
        rospy.Subscriber("event_camera_demo/event_display", EventPacket, self.new_event_packet)

    def playtimeBar_pressed(self):
        self.move_to_state(states.pause)

    def playtimeBar_released(self):
        self.move_to_state(states.back)

    def playtimeBar_dragged(self, value):
        if self.state == states.pause:
            tsec = self.recorded_data[-1]['timestamp'] - self.recorded_data[0]['timestamp']
            csec = tsec * value/1000
            self.last_frame_end = csec + self.recorded_data[0]['timestamp']
    # fucntion handles the play button being pushed
    def playbackBtn_clicked(self):
        if self.state == states.playback:
            self.move_to_state(states.pause)
        else: # only other possible state should be states.pause
            self.move_to_state(states.playback)

    # fucntion handles the record button being pushed
    def recordBtn_clicked(self):
        if self.state == states.live:
            self.move_to_state(states.record)
        elif self.state == states.record:
            self.move_to_state(states.playback)
        else: # both playback and pause states move to live state
            self.move_to_state(states.live)

    # function handles the tranistion between states
    # some buttons are only active in certain states
    # some buttons have different function is different states (their text changes)
    def move_to_state(self, new_state):
        current_state = self.state
        if new_state == states.live:
            self.recordBtn.setEnabled(True)
            self.recordBtn.setText("Record")
            self.playbackBtn.setText("Play")
            self.playtimeBar.setEnabled(False)
            self.playtimeLbl.setEnabled(False)
            self.statusLbl.setText("Live Data")
            self.playspeedBar.setEnabled(False)
            self.playspeedBox.setEnabled(False)
            self.integrationBar.setEnabled(False)
            self.integrationBox.setEnabled(False)
            self.integrationCheck.setEnabled(False)

        elif new_state == states.record:
            self.recorded_data = np.empty(0, dtype=self.event_dtype) # erase old recording to make way for new one
            self.recordBtn.setEnabled(True)
            self.recordBtn.setText("Stop")
            self.playbackBtn.setEnabled(False)
            self.playtimeLbl.setEnabled(True)
            self.playtimeLbl.setText("00:00/00:00")
            self.playtimeBar.setEnabled(False)
            self.statusLbl.setText("Live Data")
            self.playspeedBar.setEnabled(False)
            self.playspeedBox.setEnabled(False)
            self.integrationBar.setEnabled(False)
            self.integrationBox.setEnabled(False)
            self.integrationCheck.setEnabled(False)

        elif new_state == states.playback:
            self.recordBtn.setEnabled(True)
            self.recordBtn.setText("Back")
            self.playbackBtn.setEnabled(True)
            self.playbackBtn.setText("Pause")
            self.playtimeLbl.setEnabled(True)
            self.playtimeBar.setEnabled(True)
            self.statusLbl.setText("Recorded Data")
            self.playspeedBar.setEnabled(True)
            self.playspeedBox.setEnabled(True)
            if self.user_integration_interval is not None:
                self.integrationBar.setEnabled(True)
                self.integrationBox.setEnabled(True)
            self.integrationCheck.setEnabled(True)
            if not current_state == states.pause:
                self.last_frame_end = self.recorded_data[0]['timestamp']

        elif new_state == states.pause:
            self.recordBtn.setEnabled(True)
            self.recordBtn.setText("Back")
            self.playbackBtn.setEnabled(True)
            self.playbackBtn.setText("Resume")
            self.playtimeLbl.setEnabled(True)
            self.playtimeBar.setEnabled(True)
            self.statusLbl.setText("Recorded Data")
            self.playspeedBar.setEnabled(True)
            self.playspeedBox.setEnabled(True)
            if self.user_integration_interval is not None:
                self.integrationBar.setEnabled(True)
                self.integrationBox.setEnabled(True)
            self.integrationCheck.setEnabled(True)

        elif new_state == states.back:
            self.move_to_state(self.last_state)
            return

        else: # this should never happen but just in case
            self.move_to_state(states.live)
            return        
        
        # save new state
        self.last_state = current_state
        self.state = new_state

    # function for syncronising play speed bar and play speed box
    # updates box based on bar
    def speedUpdate_bar(self, value):
        self.playspeedBox.setValue(value/100)            

    # function for syncronising play speed bar and play speed box
    # updates bar based on box
    def speedUpdate_box(self, value):
        self.playspeedBar.setValue((int)(value*100))

    def integrationUpdate_bar(self, value):
        self.integrationBox.setValue(value/1000)

    def integrationUpdate_box(self, value):
        self.integrationBar.setValue((int)(value*1000))
        if self.user_integration_interval is not None:
            self.user_integration_interval = value

    def integrationUpdate_check(self, value):
        if value == 2: # 2 is checked
            self.integrationBox.setEnabled(False)
            self.integrationBar.setEnabled(False)
            self.user_integration_interval = None
        else:
            self.integrationBox.setEnabled(True)
            self.integrationBar.setEnabled(True)
            self.user_integration_interval = self.integrationBox.value()

    def update_time(self):
        cmin = 0
        csec = 0
        tmin = 0
        tsec = 0
        if len(self.recorded_data) > 0:
            tsec = self.recorded_data[-1]['timestamp'] - self.recorded_data[0]['timestamp']
            if self.last_frame_end is not None:
                csec = self.last_frame_end - self.recorded_data[0]['timestamp']
                if csec < 0:
                    csec = 0
        if self.state == states.playback:
            if tsec == 0:
                self.playtimeBar.setValue(0)
            else:
                self.playtimeBar.setValue((int)(csec/tsec*1000))
        if not tsec == 0:
            self.playtimeBar.setTickInterval((int)(1000/tsec))
        tsec = (int)(tsec)
        csec = (int)(csec)
        tmin = tsec // 60
        tsec = tsec % 60
        cmin = csec // 60
        csec = csec % 60
        self.playtimeLbl.setText("{:02.0f}:{:02.0f}/{:02.0f}:{:02.0f}".format(cmin, csec, tmin, tsec))

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
        #     self.last_frame_end = events[-1]['timestamp']
            self.image_width = data.width
            self.image_height = data.height

            
        if self.state == states.live or  self.state == states.record:
            self.display_new_frame(events)

    # calcualtes the new time range for the next frame to be displayed on screen
    # self.state == states.pause will cause this function to return the same time range (assuming playback_speed doesn't change)
    def new_frame_window(self, playback_speed):
        frame_step = 1/(self.fps/playback_speed)
        frame_width = max(frame_step, 1/(self.fps/self.min_frame_size))
        if self.user_integration_interval is None:
            self.integrationBox.setValue(frame_width)
        else:
            frame_width = self.user_integration_interval

        # only advance frame if the data is being played back (not paused)
        if self.state == states.playback:
            # move end for frame by one frame (adjusting for playback speed)
            self.last_frame_end += frame_step
            # if the end of the frame is over the end of the data
            if self.last_frame_end > self.recorded_data[-1]['timestamp'] or self.last_frame_end < self.recorded_data[0]['timestamp'] + frame_width:
                # set the end of the frame back the the start of the data (adjusting for frame width)
                self.last_frame_end = self.recorded_data[0]['timestamp'] + frame_width
                self.last_playback_indices = [0, 20000]

        return self.last_frame_end - frame_width, self.last_frame_end

    # cut one frame worth of event data out of the recorded event data and then display it
    def display_new_playback_frame(self):
        self.update_time()
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

        # get timestamps
        time_start, time_end = self.new_frame_window(playback_speed)

        # get limits for data (allowing the code to search through all of the data takes too long)
        i = max(self.last_playback_indices[0] - 100000, 0)
        j = self.last_playback_indices[1] + self.last_playback_indices[1] - self.last_playback_indices[0] + 100000

        # convert frame range in time to frame range in index
        timestamps = data['timestamp'][i:j]
        index_start = np.searchsorted(timestamps, time_start)
        index_end = np.searchsorted(timestamps, time_end, side='right')

        # correct indexes
        index_start = index_start + i
        index_end = index_end + i      

        # if the search withing the limited data didn't work the search the entire range for the correct range
        if (index_start == i and not i == 0) or (index_end == j and not j >= len(data)):
            timestamps = data['timestamp']
            index_start = np.searchsorted(timestamps, time_start)
            index_end = np.searchsorted(timestamps, time_end, side='right')

        self.last_playback_indices = [index_start, index_end]
        

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
        frame = np.ones((self.image_height, self.image_width, 3), dtype=np.uint8)*200

        # use stuff to make image
        frame[y, x, :] = 0
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
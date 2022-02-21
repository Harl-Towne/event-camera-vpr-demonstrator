from PyQt5 import QtWidgets, uic, QtCore, QtGui
import sys
import os

from cv_bridge import CvBridge
import rospy
from qcr_event_msgs.msg import EventPacket
import numpy as np
from enum import Enum
from sensor_msgs.msg import Image
from datetime import datetime
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import threading

# from eventstore import EventStore
bridge = CvBridge()

# live - showing live events from camera
# record - showing live events from camera but also saving them for later
# playback - showing previously recorded events
# pause - showing previously recorded events but not moving in time
states = Enum("states", ("live", "record", "playback", "pause", 'back'))

# main ui class
# handels main window and all events as well as the ros node and its' callbacks
class EventDemoWindow(QtWidgets.QMainWindow):

    fps = 24 # frames to display each second
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
        self.recorded_event_data = EventStore() # for recording and playing back data
        self.live_event_data = EventStore()     # for displaying live data with different integration intervals
        self.last_event_packet = None           # for displaying live data quickly with no set integration interval 
        self.user_integration_interval = None   # none is auto interval
        self.last_playback_indices = [0, 20000] # for faster searching for events to display
        self.last_live_duration = 100000        # ditto
        self.live_camera_data = None            # last camera frame (as in regular rgb camera)
        self.recorded_camera_data = {"frames":[], "timestamps":[]} # recorded camera frames
        self.last_receive_timestamp = None      # used to keep camera data in sync with events
        self.process_start_time = None          # used for preformance metric
        self.main_thread = threading.get_ident()# used to prevent segmentation faults in display_new_frame()

        # load ui
        uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "video_demo.ui"), self)

        # setup ui events
        self.playbackBtn.clicked.connect(self.playbackBtn_clicked)
        self.recordBtn.clicked.connect(self.recordBtn_clicked)
        
        self.playtimeBar.sliderPressed.connect(self.playtimeBar_pressed)
        self.playtimeBar.sliderReleased.connect(self.playtimeBar_released)
        self.playtimeBar.valueChanged.connect(self.playtimeBar_dragged)

        self.playspeedBar.valueChanged.connect(self.speedUpdate_bar)
        self.playspeedBox.valueChanged.connect(self.speedUpdate_box)

        self.integrationBar.valueChanged.connect(self.integrationUpdate_bar)
        self.integrationBox.valueChanged.connect(self.integrationUpdate_box)
        self.integrationCheck.stateChanged.connect(self.integrationUpdate_check)

        # 3d plot stuff
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.eventPlotTab.layout().addWidget(self.canvas)

        self.axes = self.figure.add_subplot(111, projection='3d')

        self.axes.set_xlabel("x Pixel Location")
        self.axes.set_ylabel("Seconds Before Present")
        self.axes.set_zlabel("y Pixel Location")

        zero = np.zeros(1)
        self.eventPlotP, = self.axes.plot(zero, zero, zero, linestyle="", marker=".", color='blue', markersize=1)
        self.eventPlotN, = self.axes.plot(zero, zero, zero, linestyle="", marker=".", color='red', markersize=1)
        
        # display refresh timer
        self.updateTimer = QtCore.QTimer()
        self.updateTimer.timeout.connect(self.display_new_playback_frame)
        self.updateTimer.start(1000//self.fps)

        
        self.updateTimer2 = QtCore.QTimer()
        self.updateTimer2.timeout.connect(self.display_new_live_frame)
        self.updateTimer2.start(1000//self.fps)

        # setup node
        rospy.init_node('event_demo_ui', anonymous=True)
        rospy.Subscriber("event_camera_demo/event_display", EventPacket, self.new_event_packet)
        rospy.Subscriber("dvs/image_raw", Image, self.new_frame)

    # when the bar is press pause the recording
    # this is needs because if the recording was playing at the same time it would fight the user
    def playtimeBar_pressed(self):
        self.move_to_state(states.pause)
    
    # when the bar is released go back to the previous state (playback or pause)
    def playtimeBar_released(self):
        self.move_to_state(states.back)

    # update the current playback time as the user moves the bar
    def playtimeBar_dragged(self, value):
        if self.state == states.pause:
            tsec = self.recorded_event_data.loc(-1)['timestamp'] - self.recorded_event_data.loc(0)['timestamp']
            csec = tsec * value/1000
            self.last_frame_end = csec + self.recorded_event_data.loc(0)['timestamp']
 
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

        elif new_state == states.record:
            self.recordBtn.setEnabled(True)
            self.recordBtn.setText("Stop")
            self.playbackBtn.setEnabled(False)
            self.playtimeLbl.setEnabled(True)
            self.playtimeLbl.setText("00:00/00:00")
            self.playtimeBar.setEnabled(False)
            self.statusLbl.setText("Live Data")
            self.playspeedBar.setEnabled(False)
            self.playspeedBox.setEnabled(False)
            # erase old recording to make way for new one
            self.recorded_event_data.wipe()
            self.recorded_camera_data['timestamps'] = []
            self.recorded_camera_data['frames'] = []

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
            if self.last_frame_end is None:
                self.last_frame_end = self.recorded_event_data.loc(0)['timestamp']
            if current_state == states.record:
                self.recorded_camera_data['timestamps'] = np.array(self.recorded_camera_data['timestamps'])

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

    # function for syncronising integration bar and integration box
    # updates box based on bar
    def integrationUpdate_bar(self, value):
        self.integrationBox.setValue(value/1000)
    
    # function for syncronising integration bar and integration box
    # updates bar based on box
    def integrationUpdate_box(self, value):
        self.integrationBar.setValue((int)(value*1000))
        if self.user_integration_interval is not None:
            self.user_integration_interval = value

    # trigger user/auto integration range when the box is ticked
    def integrationUpdate_check(self, value):
        if value == 2: # 2 is when the box is ticked
            self.integrationBox.setEnabled(False)
            self.integrationBar.setEnabled(False)
            self.user_integration_interval = None
        else:
            self.integrationBox.setEnabled(True)
            self.integrationBar.setEnabled(True)
            self.user_integration_interval = self.integrationBox.value()

    # update the current/total playback time text as well as the playtime bar
    def update_time(self):
        cmin = 0
        csec = 0
        tmin = 0
        tsec = 0

        # get total and current playback time in seconds
        if self.recorded_event_data.length() > 0: # if there is recorded data
            tsec = self.recorded_event_data.loc(-1)['timestamp'] - self.recorded_event_data.loc(0)['timestamp']
            if self.last_frame_end is not None: # if the playback hasn't started yet
                csec = self.last_frame_end - self.recorded_event_data.loc(0)['timestamp']
                if csec < 0:
                    csec = 0

        # update playtime bar position if recording is being played back
        if self.state == states.playback: 
            if tsec == 0: # avoid / by 0 errors
                self.playtimeBar.setValue(0)
            else:
                self.playtimeBar.setValue((int)(csec/tsec*1000))

        # update ticks at the bottom of the playtime bar
        if not tsec == 0:
            self.playtimeBar.setTickInterval((int)(1000/tsec))

        # calculate minutes:seconds format and display
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

        # convert ROS event_image containing events to numpy matrix
        event_packet = bridge.imgmsg_to_cv2(data.events, desired_encoding='64FC1')

        # create empty numpy array for events to be stored into (this array matches datatype used to store events)
        events = np.empty(event_packet.shape[0], dtype=self.event_dtype)

        # transfer events from numpy matrix to numpy array, dtype conversions are implicit (see event_dtype)
        for i, column in enumerate(['timestamp', 'x', 'y', 'polarity']):
            events[column] = event_packet[:, i]

        # if the data is currently being recorded also save it into the recorded_event_data array
        if self.state == states.record:
            self.recorded_event_data.append(events)

        # this if statement triggers once for the first event packet and signals to rest of the class to start generating frames
        # also saves event_image width and height (which should never change)
        self.image_width = data.width
        self.image_height = data.height

        self.last_receive_timestamp = events[-1]['timestamp']
        # if live data is currently being displayed then save that too
        if self.state == states.live or  self.state == states.record:
            if self.user_integration_interval is None:
                self.last_event_packet = events
            else:
                self.live_event_data.append(events)

    # call back for new camera image
    def new_frame(self, camera_image):
        self.live_camera_data = bridge.imgmsg_to_cv2(camera_image, desired_encoding='passthrough')
        # timestamp of the camera frame is assumed to be the same as the most recent event
        if self.state == states.record and self.last_receive_timestamp is not None:
            self.recorded_camera_data['timestamps'].append(self.last_receive_timestamp)
            self.recorded_camera_data['frames'].append(self.live_camera_data)

    # calcualtes the new time range for the next frame to be displayed on screen
    # self.state == states.pause will cause this function to return the same time range (assuming playback_speed doesn't change)
    def new_frame_window(self, playback_speed=1):
        frame_step = 1/(self.fps/playback_speed)
        # frame width set automaticaly
        frame_width = max(frame_step, 1/(self.fps/self.min_frame_size))
        # frame width either used to update integration slider
        # or set to user value
        if self.user_integration_interval is None:
            if self.state == states.playback or self.state == states.pause:
                self.integrationBox.setValue(frame_width)
        else:
            frame_width = self.user_integration_interval

        # only advance frame if the data is being played back (not paused)
        if self.state == states.playback:
            # move end for frame by one frame (adjusting for playback speed)
            self.last_frame_end += frame_step
            # if the end of the frame is over the end of the data
            if self.last_frame_end > self.recorded_event_data.loc(-1)['timestamp'] or self.last_frame_end < self.recorded_event_data.loc(0)['timestamp'] + frame_width:
                # set the end of the frame back the the start of the data (adjusting for frame width)
                self.last_frame_end = self.recorded_event_data.loc(0)['timestamp'] + frame_width
                self.last_playback_indices = [0, 20000]
        
        if self.state == states.playback or self.state == states.pause:
            return self.last_frame_end - frame_width, self.last_frame_end
        else:
            # if live data is being displayed then the end of the frame window is set to the end of the live data.
            # this keeps the display in sync with the data coming in.
            live_end = self.live_event_data.loc(-1)['timestamp']
            return live_end - frame_width, live_end

    # called on a timer
    # gets one frame of data from the stored live data and then displays it
    def display_new_live_frame(self):
        # save state just in case it changes while this functon is running
        # it shouldn't, but just in case
        state = self.state
        if not (state == states.live or state == states.record):
            return
        
        # if the user has not set their own integration interval then this function is not needed (but is still run on a timer)
        if self.user_integration_interval is None:
            self.display_new_frame(self.last_event_packet)
            return

        # abort if no data
        # this happens with the live playback sometimes
        # not sure why
        if self.live_event_data.length() == 0:
            return

        self.process_start_time = datetime.now()
        
        # get timestamps
        time_start, time_end = self.new_frame_window()

        # get limits for data (allowing the code to search through all of the data takes too long)
        j = self.live_event_data.length()-1
        i = j - self.last_live_duration - 10000
        i = max(i, 0)

        data = self.live_event_data.loc(i, j)

        # convert frame range in time to frame range in index
        timestamps = data['timestamp']#[i:j]
        index_start = np.searchsorted(timestamps, time_start)
        index_end = j
        
        # if the search withing the limited data didn't work the search the entire range for the correct range
        if (index_start == 0 and not i == 0) or (index_end == j-i+1 and not j >= len(data)):
            data = self.live_event_data.loc(0, -1)
            timestamps = data['timestamp']
            index_start = np.searchsorted(timestamps, time_start)

        self.last_live_duration = index_end - index_start

        # get just the data from this frame
        data = data[index_start:index_end+1]

        self.display_new_frame(data)
        try:
            # sometimes this errors out, no clue why
            self.live_event_data.clean(timestamps[index_start])
        except:
            pass
        
    # called on a timer
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
            # data = self.recorded_event_data.loc(0, -1)
            playback_speed = self.playspeedBox.value()
        else:
            # don't display new data if live data is being displayed
            return

        # abort if no data
        # this happens with the live playback sometimes
        # not sure why
        if self.recorded_event_data.length == 0:
            return

        self.process_start_time = datetime.now()

        # get timestamps
        time_start, time_end = self.new_frame_window(playback_speed)

        # get limits for data (allowing the code to search through all of the data takes too long)
        i = max(self.last_playback_indices[0] - 100000, 0)
        j = self.last_playback_indices[1] + self.last_playback_indices[1] - self.last_playback_indices[0] + 100000

        data = self.recorded_event_data.loc(i, j)
        # convert frame range in time to frame range in index
        timestamps = data['timestamp']#[i:j]
        index_start = np.searchsorted(timestamps, time_start)
        index_end = np.searchsorted(timestamps, time_end, side='right')

        # update for use in next data limits
        self.last_playback_indices = [index_start + i, index_end + i] 

        # if the search withing the limited data didn't work the search the entire range for the correct range
        if (index_start == 0 and not i == 0) or (index_end == j-i+1 and not j >= len(data)):
            data = self.recorded_event_data.loc(0, -1)
            timestamps = data['timestamp']
            index_start = np.searchsorted(timestamps, time_start)
            index_end = np.searchsorted(timestamps, time_end, side='right')
            # update for use in next data limits
            self.last_playback_indices = [index_start, index_end] 
        
        # get just the data from this frame
        data = data[index_start:index_end+1]

        # display the data
        self.display_new_frame(data)

    # display the given event data as a frame
    def display_new_frame(self, event_data):
        # prevents a nonetype error
        if event_data is None:
            return

        # prevents segmentation faults due to qt being accessed from other threads
        if not threading.get_ident() == self.main_thread:
            print("'display_new_frame' function called from wrong thread.")
            return

        # for timing the process
        if self.process_start_time is None:
            self.process_start_time = datetime.now()
    
        x = event_data['x']
        y = event_data['y']
        p = event_data['polarity']

        if self.tabWidget.currentIndex() == 1 or self.tabWidget.currentIndex() == 2:

            # remove offset from timestamps
            ts = (event_data['timestamp'] - event_data['timestamp'][-1])

            # flip time axis so that the past is positive time
            ts = ts * -1

            # filter x/y/ts by polarity
            bool_p = np.array(p.copy(), dtype=bool)
            xp = x[bool_p] 
            xn = x[np.invert(bool_p)] 
            yp = y[bool_p]
            yn = y[np.invert(bool_p)]
            tp = ts[bool_p]
            tn = ts[np.invert(bool_p)]

            # plot data            
            self.eventPlotP.set_data(xp, tp)
            self.eventPlotP.set_3d_properties(yp)

            self.eventPlotN.set_data(xn, tn)
            self.eventPlotN.set_3d_properties(yn)

            # set plot limits
            self.axes.set_xlim3d(np.min(x), np.max(x))
            self.axes.set_ylim3d(np.min(ts), np.max(ts))
            self.axes.set_zlim3d(np.min(y), np.max(y))

            # flip the plot verticaly as pixel coordinates are measured downwards as positive
            # the plot doesn't need to be fliped horizontaly because the time axis was already flipped
            self.axes.invert_zaxis()    

            self.figure.canvas.draw()

        else:
            
            # get camera frame to display
            camera_frame = None
            if self.state == states.live or self.state == states.record:
                # most recent frame if displaying live
                camera_frame = self.live_camera_data
            elif len(self.recorded_camera_data['timestamps']) > 0 and len(event_data['timestamp']) > 0: # sometimes this function will be asked to display an empty event frame
                # frame most closely lining up wit the start of the frame window if displaing recorded
                target_ts = event_data['timestamp'][0]
                i = np.searchsorted(self.recorded_camera_data['timestamps'], target_ts)
                if i >= len(self.recorded_camera_data['frames']):
                    i = len(self.recorded_camera_data['frames']) - 1
                camera_frame = self.recorded_camera_data['frames'][i]


            # initialise frame for event display
            if self.overlayCheck.checkState() == 2 and camera_frame is not None:
                # background is camera frame
                frame = camera_frame.copy()
            else:
                if not self.fastCheck.checkState() == 2:
                    # background is light grey
                    frame = np.ones((self.image_height, self.image_width, 3), dtype=np.uint8)*200
                else:
                    # background is black
                    # is black background is needed if "frame[y, x, :] = 0" line will be skipped later for faster event displays
                    frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

            # add events to image
            if not self.fastCheck.checkState() == 2:
                # skip this for faster event display as it can take a long time in the case of many events
                try:
                    frame[y, x, :] = 0
                except:
                    print("="*40)
                    print(frame.shape)
                    print(x.shape)
                    print(y.shape)
                    print(max(x), max(y))
            frame[y, x, p * 2] = 255

            # convert frrame to pixmap
            # don't ask about the self.image_width*3. I don't know what it does but it's important
            # event_image (along with a few other variables here) are clas variables to try and prevent segmentation faults
            self.event_image = QtGui.QImage(frame.copy(), self.image_width, self.image_height, self.image_width*3, QtGui.QImage.Format_RGB888) # convert np event_image to qt event_image
            event_pixmap = QtGui.QPixmap.fromImage(self.event_image)

            # calculate appropriate width for images
            # the -15 leave a border. this needs to be >0 otherwise the window will uncontrolably expand.
            # The expanding is because the frames width is often a couple of pixels wider/higher than the combined widths of all its children
            width = self.displayFrame.width() - 15
            height = self.displayFrame.height() - 15
            scaled_width = (int)(height * self.image_width / self.image_height)

            if self.cameraCheck.checkState() == 2:
                # set width of each display to half
                scaled_width = min((int)(width/2), scaled_width)
                # display camera frame if it exists
                if camera_frame is not None:
                    self.camera_image = QtGui.QImage(camera_frame.copy(), self.image_width, self.image_height, self.image_width*3, QtGui.QImage.Format_RGB888) # conver np event_image to qt event_image
                    camera_pixmap = QtGui.QPixmap(self.camera_image)
                    self.camera_pixmap_scaled = camera_pixmap.scaled((int)(scaled_width),self.frameDisplay.height(), QtCore.Qt.KeepAspectRatio) # scale event_pixmap
                    self.frameDisplay.setPixmap(self.camera_pixmap_scaled) # display event_pixmap
                self.frameDisplay.setMinimumWidth((int)(scaled_width))
            else:
                # set width of each display to full
                scaled_width = min(width, scaled_width)

                # clear other display and set its width to zero
                self.frameDisplay.clear() # display event_pixmap
                self.frameDisplay.setMinimumWidth(0)

            # display the event frame
            self.eventDisplay.setMinimumWidth((int)(scaled_width))
            self.event_pixmap_scaled = event_pixmap.scaled(scaled_width,self.eventDisplay.height(), QtCore.Qt.KeepAspectRatio) # scale event_pixmap
            self.eventDisplay.setPixmap(self.event_pixmap_scaled) # display event_pixmap       

        # display timing for drawing process
        performance = datetime.now()-self.process_start_time
        performance = performance.total_seconds()
        performance = (1/self.fps) / performance * 100

        message = "Displaying at {:4.0f}% of Real Time".format(performance)
        self.speedLbl.setText(message)
        self.process_start_time = None

# EventStore class is intended for storing large quantaties of events, in the form of numpy arrays, quickly.
# It stores each numpy matrix containing a set of events in a list so that joining them together isn't required.
# The numpy matricies are then access as if there were joined, the .loc function will always return one matrix no matter how many matricies the result spaned
# I cannot for the life of me manage to import this from another file so it's in here instead
class EventStore():

    def __init__(self):
        self.event_packets = []

    # add new packet to end of data
    def append(self, packet):
        self.event_packets.append(packet)

    # add new packet to start of data
    def prepend(self, packet):
        self.event_packets.insert(0, packet)

    # count total number of events
    def length(self):
        s = 0
        for packet in self.event_packets:
            s = s + len(packet)
        return s

    # return data specified. parameters can be : 1 string, 1 integer or 2 integers
    # 1 string: return parameter matching string from all events ('x', 'y', 'timestamp', 'polarity)
    # 1 integer: return event matching index
    # 2 integers: return events within integer range (inclusive, exclusive)
    # indexes should be given as if the data was one big array
    def loc(self, *args): #two indexes is range
        length = self.length()
        if length == 0: # no data
            raise Exception("No data")

        if len(args) == 0: # wrong parameters
            raise Exception("Give index")

        # string index (select column)
        if len(args) == 1 and type(args[0]) == str:
            k = args[0] # k for kolumn
            ret = np.empty(length, dtype=self.event_packets[0][k].dtype)
            i = 0
            for packet in self.event_packets:
                j = i + len(packet)
                ret[i:j] = packet[k]
                i = j
            return ret
        # not string index (clamp numbers)
        else:
            if len(args) > 0:
                a = args[0]
                if a < 0:
                    a = length + a
                elif a >= length:
                    a = length - 1 # this -1 stops the last element ever being accessed but it prevents a bug that I don't want to fix
            if len(args) > 1:
                b = args[1]
                if b <= 0:
                    b = length + b
                elif b >= length:
                    b = length - 1 # this -1 stops the last element ever being accessed but it prevents a bug that I don't want to fix

        # one integer index 
        if len(args) == 1:
            i, j = self.iloc(a)
            return self.event_packets[i][j]

        # two integer indices
        elif len(args) > 1:
            # get start and end data points
            i1, j1 = self.iloc(a)
            i2, j2 = self.iloc(b)
            # get start and end data (likely part of a larger array)
            start = self.event_packets[i1][j1:]
            end = self.event_packets[i2][:j2]
            # count length of data to be returned
            length = len(start) + len(end)
            for packet in self.event_packets[i1+1:i2]:
                length = length + len(packet)
            # initialise return array
            ret = np.empty(length, dtype=self.event_packets[i1][j1].dtype)
            # fill out return array
            i = 0
            j = i + len(start)
            ret[i:j] = start
            for packet in self.event_packets[i1+1:i2]:
                i = j
                j = i + len(packet)
                ret[i:j] = packet
            i = j
            j = i + len(end)
            ret[i:j] = end
            return ret
        
        # if nothing has been returned yet then it must be the users fault
        raise IndexError("Invalid indices")

    # locate the relative indexes from global index
    def iloc(self, global_index):
        j = global_index
        # loop through all packets, decreasing j by the length of each packet, until j specifies a event in a packet
        # this could probably be improved by indexing the length of the packets when the are added or something but for now it seems to run plenty fast enough
        i = 0
        while j >= len(self.event_packets[i]):
            j = j - len(self.event_packets[i])
            i = i + 1
            if i >= len(self.event_packets):
                raise IndexError("Index {} out of range {}.".format(global_index, self.length()))
        return i, j

    # delete all data
    def wipe(self):
        self.event_packets = []

    # remove all data before the specified timestamp
    def clean(self, cuttoff):
        while self.length() > 0 and self.event_packets[0][-1]['timestamp'] < cuttoff:
            self.event_packets.pop(0)


if __name__ == '__main__':
    # start GUI
    app = QtWidgets.QApplication(sys.argv)
    window = EventDemoWindow()
    window.show()
    sys.exit(app.exec())
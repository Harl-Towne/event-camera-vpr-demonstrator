from PyQt5 import QtWidgets, uic
import sys
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        # load ui
        uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "video_demo.ui"), self)

if __name__ == '__main__':
    # start GUI
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
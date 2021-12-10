from nodes import event_to_visualiser_2d
from PyQt5 import QtWidgets, uic
import sys
import os

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        print(os.path.dirname(os.path.realpath(__file__)))
        uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "video_demo.ui"), self)

if __name__ == '__main__':
    event_to_visualiser_2d.start_node(False)

    # start GUI
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
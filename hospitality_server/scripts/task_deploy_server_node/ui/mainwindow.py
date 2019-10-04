from PyQt5 import QtWidgets, QtCore, QtGui

import rospy
from hospitality_msgs.srv import TaskReq, TaskReqRequest, TaskReqResponse

if __name__ == "__main__":
    from elements.service_selector_widget import *
else:
    from .elements.service_selector_widget import *

class MainWindow(QtWidgets.QMainWindow):
    resized = QtCore.pyqtSignal()
    def __init__(self):
        super(MainWindow, self).__init__()
        
        self.setWindowTitle("Real-Time Task Dispatcher")
        self.setGeometry(50, 50, 960, 540)

        self.service_selector_widget = ServiceSelectorWidget(parent=self)

        self.connect_signals()
        self.show()
    
    def connect_signals(self):
        self.service_selector_widget.receive_delivery_tab.confirmed.connect(self.receive_delivery_handler)
        self.service_selector_widget.send_delivery_tab.confirmed.connect(self.send_delivery_handler)
        self.resized.connect(self.resize_ui)

    def receive_delivery_handler(self):
        destination, obj, urgency = self.service_selector_widget.get_inbound_delivery_task()
        destination = int(destination[:destination.index(":")].strip())
        print "Configuring task with destination %d, object %s, and urgency %d" % (destination, obj, urgency)
    
    def send_delivery_handler(self):
        print "send delivery"

    def resizeEvent(self, event):
        self.resized.emit()
        return super(MainWindow, self).resizeEvent(event)
    
    def resize_ui(self):
        left_pos = 15
        self.service_selector_widget.move(left_pos, 10)
        self.service_selector_widget.resize(self.width() - 30, 250)

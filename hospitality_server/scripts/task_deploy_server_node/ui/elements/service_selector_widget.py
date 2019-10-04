from PyQt5 import QtWidgets, QtGui, QtCore
from .util_widgets.radiobox_array import *
from .delivery_tabs.receive_delivery_tab import ReceiveDeliveryTab
from .delivery_tabs.send_delivery_tab import SendDeliveryTab

import csv
import rospkg

class ServiceSelectorWidget(QtWidgets.QWidget):
    def __init__(self, parent):
        super(ServiceSelectorWidget, self).__init__(parent=parent)

        self.tab_widget = QtWidgets.QTabWidget(parent=self)
        self.receive_delivery_tab = ReceiveDeliveryTab(parent=self.tab_widget)
        self.send_delivery_tab = SendDeliveryTab(parent=self.tab_widget)
        self.tab_widget.addTab(self.receive_delivery_tab, "Receive Delivery")
        self.tab_widget.addTab(self.send_delivery_tab, "Send Delivery")

        self.connect_signals()
    
    def connect_signals(self):
        pass
    
    def get_inbound_delivery_task(self):
        destination = self.receive_delivery_tab.dest_combobox.currentText()
        obj = self.receive_delivery_tab.object_combobox.currentText()
        urgency = self.receive_delivery_tab.urgency_radio.get_selection()
        return destination, obj, urgency
    
    def resize(self, width, height):
        super(ServiceSelectorWidget, self).resize(width, height)

        tab_bar_height = self.tab_widget.tabBar().height()
        tab_area_height = self.height() - tab_bar_height
        
        self.tab_widget.move(0, 0)
        self.tab_widget.resize(self.width(), tab_area_height + tab_bar_height)
        
        self.receive_delivery_tab.move(10, 10)
        self.receive_delivery_tab.resize(self.tab_widget.width() - 20, tab_area_height - 20)

        self.send_delivery_tab.move(10, 10)
        self.send_delivery_tab.resize(self.tab_widget.width() - 20, tab_area_height - 20)
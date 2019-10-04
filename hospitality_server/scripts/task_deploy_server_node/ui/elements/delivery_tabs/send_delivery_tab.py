import csv
from PyQt5 import QtCore, QtGui, QtWidgets

import rospkg

from ..util_widgets.radiobox_array import HorizontalRadioButtonArray

class SendDeliveryTab(QtWidgets.QWidget):
    confirmed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(SendDeliveryTab, self).__init__(parent=parent)
        self.start_label = QtWidgets.QLabel(parent=self, text="Your Location")
        self.start_label.setAlignment(QtCore.Qt.AlignVCenter)
        self.start_combobox = QtWidgets.QComboBox(parent=self)

        self.dest_label = QtWidgets.QLabel(parent=self, text="Destination")
        self.dest_label.setAlignment(QtCore.Qt.AlignVCenter)
        self.dest_combobox = QtWidgets.QComboBox(parent=self)

        self.urgency_label = QtWidgets.QLabel(parent=self, text="Urgency Rating")
        labels = ["Emergency", "Urgent", "Usual", "Idle"]
        self.urgency_radio = HorizontalRadioButtonArray(parent=self, label_list=labels, row_len=4)

        self.confirm_button = QtWidgets.QPushButton(parent=self, text="Confirm\nOutbound\nDelivery")

        csv_path = rospkg.RosPack().get_path("hospitality_server") + "/data/map/csv/places.csv"
        first_line = True
        with open(csv_path, "r") as csv_fp:
            csv_reader = csv.reader(csv_fp)
            for row in csv_reader:
                if first_line:
                    first_line = False
                    continue
                if row[1] != "-":
                    self.start_combobox.addItem(row[0] + ": " + row[1])
                    self.dest_combobox.addItem(row[0] + ": " + row[1])

        self.connect_signals()
        
    def connect_signals(self):
        self.confirm_button.clicked.connect(self.confirm_button_handler)
    
    def confirm_button_handler(self):
        self.confirmed.emit()
    
    def resize(self, width, height):
        super(SendDeliveryTab, self).resize(width, height)

        hmargin, vmargin = 10, 10
        hspace, vspace = 10, 20
        height = 30
        label_width = 100
        button_width = 100
        combobox_width = self.width() - 2 * hmargin - 3 * hspace - label_width - button_width
        top_pos = 10
        
        left_pos = hmargin
        top_pos = vmargin
        self.start_label.move(left_pos, top_pos)
        self.start_label.resize(label_width, height)

        left_pos += label_width + hspace
        self.start_combobox.move(left_pos, top_pos)
        self.start_combobox.resize(combobox_width, height)

        left_pos = hmargin
        top_pos += height + vspace
        self.dest_label.move(left_pos, top_pos)
        self.dest_label.resize(label_width, height)

        left_pos += label_width + hspace
        self.dest_combobox.move(left_pos, top_pos)
        self.dest_combobox.resize(combobox_width, height)

        left_pos = hmargin
        top_pos += height + vspace
        self.urgency_label.move(left_pos, top_pos)
        self.urgency_label.resize(label_width, height)

        left_pos += label_width + hspace
        self.urgency_radio.move(left_pos, top_pos)
        self.urgency_radio.resize(combobox_width, height)

        left_pos += combobox_width + hspace
        top_pos = vmargin
        self.confirm_button.move(left_pos, top_pos)
        self.confirm_button.resize(button_width, self.height() - 2 * vmargin)

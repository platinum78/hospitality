from PyQt5 import QtWidgets, QtGui, QtCore


class RadioButtonArray(QtWidgets.QWidget):
    selected = QtCore.pyqtSignal()

    def __init__(self, parent, label_list, line_len):
        super(RadioButtonArray, self).__init__(parent=parent)
        self.items = len(label_list)
        self.line_len = line_len
        self.full_lines = int(self.items / self.line_len)
        self.non_full_lines = 1 if self.items % self.line_len != 0 else 0

        self.radio_array = [[QtWidgets.QRadioButton(parent=self) for idx in range(self.line_len)] for line in range(self.full_lines)]
        if self.non_full_lines != 0:
            self.radio_array.append([QtWidgets.QRadioButton(parent=self) for idx in range(self.items - self.line_len * self.full_lines)])
        
        label_idx = 0
        for row in range(len(self.radio_array)):
            for col in range(len(self.radio_array[row])):
                self.radio_array[row][col].setText(label_list[label_idx])
                label_idx += 1
        
        self.connect_signal()
        
    def connect_signal(self):
        for c in range(len(self.radio_array)):
            for r in range(len(self.radio_array[c])):
                self.radio_array[c][r].toggled.connect(self.selection_handler)
    
    def get_selection(self):
        label_idx = 0
        for line in range(len(self.radio_array)):
            for idx in range(len(self.radio_array[line])):
                if self.radio_array[line][idx].isChecked():
                    return label_idx
                label_idx += 1
    
    def selection_handler(self):
        self.selected.emit()


class HorizontalRadioButtonArray(RadioButtonArray):
    def __init__(self, parent, label_list, row_len):
        super(HorizontalRadioButtonArray, self).__init__(parent=parent, label_list=label_list, line_len=row_len)

    def resize(self, width, height):
        super(HorizontalRadioButtonArray, self).resize(width, height)
        hmargin = 5
        col_width = int(self.width() / self.line_len)
        radio_width = col_width - 2 * hmargin

        row_height = int(self.height() / (self.full_lines + self.non_full_lines))
        vmargin = int((row_height - 30) / 2)

        for row in range(len(self.radio_array)):
            for col in range(len(self.radio_array[row])):
                self.radio_array[row][col].resize(radio_width, 30)
                self.radio_array[row][col].move(col_width * col + hmargin, row_height * row + vmargin)


class VerticalRadioButtonArray(RadioButtonArray):
    def __init__(self, parent, label_list, col_len):
        super(VerticalRadioButtonArray, self).__init__(parent=parent, label_list=label_list, line_len=col_len)
    
    def resize(self, width, height):
        super(VerticalRadioButtonArray, self).resize(width, height)
        hmargin = 5
        col_width = int(self.width() / (self.full_lines + self.non_full_lines))
        radio_width = col_width - 2 * hmargin

        row_height = int(self.height() / self.line_len)
        vmargin = int((row_height - 30) / 2)

        for col in range(len(self.radio_array)):
            for row in range(len(self.radio_array[col])):
                self.radio_array[col][row].resize(radio_width, 30)
                self.radio_array[col][row].move(col_width * col + hmargin, row_height * row + vmargin)
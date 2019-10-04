import os, sys
from PyQt5 import QtWidgets

if __name__ == "__main__":
    from ui.mainwindow import MainWindow
else:
    from .ui.mainwindow import MainWindow

def main():
    app = QtWidgets.QApplication([])
    window = MainWindow()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
#!/usr/bin/python


from PySide.QtCore import *
from PySide.QtGui import *

import numpy as np

from PySide import QtGui 

import MainWindow
import sys
 #itade address


if __name__ == '__main__':
    # Create the Qt Application
    app = QtGui.QApplication(sys.argv)
    init=MainWindow.MainWindow()
    # Run the main Qt loop
    sys.exit(app.exec_())

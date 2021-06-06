# !/usr/bin/python
import sys
from datetime import datetime

from PyQt5.QtCore import pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel

from Target import targets


class GUI(QWidget):
    trigger = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.trigger.connect(self.handleupdate)
        self.grid = QGridLayout()
        self.headings = ["Device", "Name", "fCnt", "Elapsed", "RSSI", "SNR", "DR", "Lat,Long", "Age", "Tracking"]
        for i, h in enumerate(self.headings):
            w = QLabel(h)
            w.setStyleSheet("font-weight: bold")
            self.grid.addWidget(w, 0, i)

        self.rows = {}
        self.setLayout(self.grid)
        self.setGeometry(300, 300, 300, 220)
        self.setWindowTitle('Playa Tracker Devices')
        self.numrows = 0
        self.show()
        # Update at regular intervals so elapsed times are good
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.handleupdate)
        self.timer.start(1000)

    def updateDeviceRow(self, device):
        if device not in self.rows:
            # Add a new device
            self.numrows += 1
            self.rows[device] = [QLabel() for _ in self.headings]
            for i, w in enumerate(self.rows[device]):
                self.grid.addWidget(w, self.numrows, i)
        target = targets[device]
        row = self.rows[device]
        row[0].setText(f"{device}")
        row[1].setText(target.deviceName)  # Set the value for this device
        row[2].setText(f"{target.fCnt}")
        if target.lastmsgtime is not None:
            elapsed = (datetime.utcnow() - target.lastmsgtime).total_seconds()
            row[3].setText("%.0f" % elapsed)
        if target.rxInfo is not None:
            row[4].setText("%.0f" % target.rxInfo['rssi'])
            row[5].setText("%.0f" % target.rxInfo['loRaSNR'])
        if target.txInfo is not None:
            row[6].setText("%d" % target.txInfo['dr'])

        loc = targets[device].getLocation()
        if loc[0] is not None:
            age = (datetime.utcnow() - loc[2]).total_seconds()
            row[7].setText("%f,%f" % (loc[0], loc[1]))  # Set the value for this device
            row[8].setText("%.0f" % age)  # Set the value for this device
        row[9].setText("%d" % target.tracking)

    @pyqtSlot()
    def handleupdate(self):
        for device in targets:
            self.updateDeviceRow(device)


app = QApplication(sys.argv)
gui = GUI()

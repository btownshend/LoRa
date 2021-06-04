# !/usr/bin/python
import json
import sys

import paho.mqtt.client as mqtt
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QPlainTextEdit


class TargetData:
    def __init__(self):
        self.values = {}

    def set(self, values):
        self.values = values


allTargets = {}


class Example(QWidget):
    trigger = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.initUI()
        self.initMQTT()
        self.trigger.connect(self.handleupdate)

    def initUI(self):
        self.grid = QGridLayout()
        self.grid.addWidget(QLabel("Device"), 0, 0)
        self.grid.addWidget(QLabel("Data"), 0, 1)
        self.rows = {}
        self.setLayout(self.grid)
        self.setGeometry(300, 300, 300, 220)
        self.setWindowTitle('MQTT Viewer')
        self.numrows = 0
        self.show()

    @pyqtSlot()
    def handleupdate(self):
        print(f"handleupdate")
        for device in allTargets:
            if device not in self.rows:
                # Add a new device
                self.numrows += 1
                lbl = QLabel(device)
                val = QPlainTextEdit()
                self.rows[device] = (lbl, val)
                self.grid.addWidget(lbl, self.numrows, 0)
                self.grid.addWidget(val, self.numrows, 1)
            self.rows[device][1].setPlainText(str(allTargets[device]))  # Set the value for this device

    # The callback for when the client receives a CONNACK response from the server.
    # noinspection PyUnusedLocal
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        # client.subscribe("$SYS/#")
        client.subscribe("application/#")

    # The callback for when a PUBLISH message is received from the server.
    # noinspection PyUnusedLocal
    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.payload))
        j = json.loads(msg.payload)
        print(f"j={j}")
        device = j['deviceName']
        if 'object' in j:
            allTargets[device] = j['object']
            self.trigger.emit()

    def initMQTT(self):
        client = mqtt.Client()
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.connect("192.168.0.246", 1883, 60)
        # Start the MQTT handler loop
        client.loop_start()


def main():
    app = QApplication(sys.argv)
    # noinspection PyUnusedLocal
    ex = Example()
    print("Starting main event loop")
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

# !/usr/bin/python
import base64
import json
import sys

import paho.mqtt.client as mqtt
from pymongo import MongoClient
from datetime import datetime

from GUI import gui, app
from Target import targets, Target


class PlayaTracker:
    def __init__(self, appID):
        super().__init__()
        self.on_message_gateway_event = None
        self.uartrx = ""
        self.client = None
        self.db = None
        self.mqttdata = None
        self.gwloc = None  # Location of gateway
        self.targetMap = []  # Map from target index (1..7) to devEUI
        self.appID = appID
        self.initMongo()
        self.initMQTT()

    def start(self):
        # Start the MQTT handler loop
        self.client.loop_start()

    def initMongo(self):
        self.client = MongoClient("mongodb+srv://admin:admin@test.7eofe.mongodb.net/Test?retryWrites=true&w=majority")
        print(self.client)
        self.db = self.client.Test
        print(self.db)
        self.mqttdata = self.db.mqttdata
        print(self.mqttdata)

    def sendmessage(self, devEUI, fPort, payload, confirmed=False):
        topic = f"application/{self.appID}/device/{devEUI}/tx"
        msg = {'confirmed': confirmed, 'fPort': fPort, 'data': base64.b64encode(payload).decode('utf-8') }
        payload = json.dumps(msg)
        print(f"sendmessage: topic={topic}, payload={payload}")
        (rc,mid)=self.client.publish(topic, payload=payload)
        if rc!= mqtt.MQTT_ERR_SUCCESS:
            print(f"Error sending message to {topic}: {rc}")
        else:
            print(f"sent message with mid={mid}")

    def rxmessage(self, msg):
        # Update current location of device, and what it is tracking (if any)
        devEUI = msg['devEUI']
        if devEUI not in targets:
            print(f"Adding new devEUI: {devEUI}")
            targets[devEUI] = Target(self, devEUI)
        targets[devEUI].update(msg)
        gui.trigger.emit()

    # The callback for when the client receives a CONNACK response from the server.
    # noinspection PyUnusedLocal,PyMethodMayBeStatic
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        # client.subscribe("$SYS/#")
        client.subscribe("#")

    # noinspection PyUnusedLocal
    def on_message(self, client, userdata, msg):
        if mqtt.topic_matches_sub('uart/rx', msg.topic):
            self.uartrx += msg.payload.decode('utf-8')
            left = self.uartrx.find('{')
            right = self.uartrx.find('}')
            if left != -1 and right != -1 and right > left:
                try:
                    j = json.loads(self.uartrx[left:right + 1])
                    self.uartrx = self.uartrx[right + 1:]
                except json.JSONDecodeError as ex:
                    print(f"{self.uartrx[left:right + 1]} not JSON")
                    self.uartrx = self.uartrx[right + 1:]
                    return
            else:
                if left > 0:
                    self.uartrx = self.uartrx[left:]
                return
        else:
            try:
                j = json.loads(msg.payload)
            except json.JSONDecodeError as ex:
                print(f"{msg.topic} not JSON")
                return
            except UnicodeDecodeError as ex:
                print(f"{msg.topic} not unicode")
                return
        j['topic'] = msg.topic

        if mqtt.topic_matches_sub('application/2/device/+/rx', msg.topic):
            self.rxmessage(j)
            collection = self.db.rx
        elif mqtt.topic_matches_sub('application/2/device/+/tx', msg.topic):
            print("Ignore tx message")
            return
        elif mqtt.topic_matches_sub('application/2/device/+/status', msg.topic):
            collection = self.db.status
        elif mqtt.topic_matches_sub('gateway/+/event/stats', msg.topic):
            collection = self.db.gwstats
        elif mqtt.topic_matches_sub('gateway/+/event/#', msg.topic):
            collection = self.db.gwevent
        elif mqtt.topic_matches_sub('gateway/+/command/#', msg.topic):
            collection = self.db.gwcommand
        elif mqtt.topic_matches_sub('uart/tx', msg.topic):
            collection = self.db.uarttx
        elif mqtt.topic_matches_sub('uart/rx', msg.topic):
            print(j)
            collection = self.db.uartrx
        else:
            print(f"Unrecognized topic: {msg.topic}")
            collection = self.db.other
        result = collection.insert_one(j)
        print(f"{msg.topic} inserted into {collection.name}")

    def initMQTT(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("192.168.0.246", 1883, 60)
        print('initMQTT done')


def main():
    pt = PlayaTracker(2)
    pt.start()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

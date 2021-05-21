# !/usr/bin/python
import json
import time

import paho.mqtt.client as mqtt
from pymongo import MongoClient


class MQTT2Mongo:
    def __init__(self):
        super().__init__()
        self.on_message_gateway_event = None
        self.initMongo()
        self.initMQTT()

    def run(self):
        # Start the MQTT handler loop
        self.client.loop_forever()

    def initMongo(self):
        self.client = MongoClient("mongodb+srv://admin:admin@test.7eofe.mongodb.net/Test?retryWrites=true&w=majority")
        print(self.client)
        self.db=self.client.Test
        print(self.db)
        self.mqttdata=self.db.mqttdata
        print(self.mqttdata)

    # The callback for when the client receives a CONNACK response from the server.
    # noinspection PyUnusedLocal
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        #client.subscribe("$SYS/#")
        client.subscribe("#")

    # noinspection PyUnusedLocal
    def on_message(self, client, userdata, msg):
        j = json.loads(msg.payload)
        j['topic']=msg.topic
        if msg.mid != 0:
            j['_id']=msg.mid
            print(f"mid={mid}")
        if mqtt.topic_matches_sub('application/2/device/+/rx',msg.topic):
            result=self.db.rx.insert_one(j)
        elif mqtt.topic_matches_sub('application/2/device/+/status',msg.topic):
            result=self.db.status.insert_one(j)
        elif mqtt.topic_matches_sub('gateway/+/event/stats',msg.topic):
            result=self.db.gwstats.insert_one(j)
        elif mqtt.topic_matches_sub('gateway/+/event/#',msg.topic):
            result=self.db.gwevent.insert_one(j)
        elif mqtt.topic_matches_sub('gateway/+/command/#',msg.topic):
            result=self.db.gwcommand.insert_one(j)
        else:
            print(f"Unrecognized topic: {msg.topic}")
            result=self.db.other.insert_one(j)
            return
        print(f"{msg.topic} -> {result}")

    def initMQTT(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("192.168.0.246", 1883, 60)



def main():
    ex = MQTT2Mongo()
    ex.run()

if __name__ == '__main__':
    main()

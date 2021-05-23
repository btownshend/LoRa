# !/usr/bin/python
import json
import time
from json import JSONDecodeError

import paho.mqtt.client as mqtt
from pymongo import MongoClient


class MQTT2Mongo:
    def __init__(self):
        super().__init__()
        self.on_message_gateway_event = None
        self.uartrx=""
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
        if mqtt.topic_matches_sub('uart/rx',msg.topic):
            self.uartrx+=msg.payload.decode('utf-8')
            left=self.uartrx.find('{')
            right=self.uartrx.find('}')
            if left!=-1 and right!=-1 and right>left:
                try:
                    j=json.loads(self.uartrx[left:right+1])
                    self.uartrx=self.uartrx[right+1:]
                except JSONDecodeError as ex:
                    print(f"{self.uartrx[left:right+1]} not JSON")
                    self.uartrx=self.uartrx[right+1:]
                    return
            else:
                if left>0:
                    self.uartrx=self.uartrx[left:]
                return
        else:
            try:
                j = json.loads(msg.payload)
            except JSONDecodeError as ex:
                print(f"{msg.topic} not JSON")
                return
        j['topic']=msg.topic
        if msg.mid != 0:
            j['_id']=msg.mid
            print(f"mid={mid}")
        if mqtt.topic_matches_sub('application/2/device/+/rx',msg.topic):
            collection=self.db.rx
        elif mqtt.topic_matches_sub('application/2/device/+/status',msg.topic):
            collection=self.db.status
        elif mqtt.topic_matches_sub('gateway/+/event/stats',msg.topic):
            collection=self.db.gwstats
        elif mqtt.topic_matches_sub('gateway/+/event/#',msg.topic):
            collection=self.db.gwevent
        elif mqtt.topic_matches_sub('gateway/+/command/#',msg.topic):
            collection=self.db.gwcommand
        elif mqtt.topic_matches_sub('uart/tx',msg.topic):
            collection=self.db.uarttx
        elif mqtt.topic_matches_sub('uart/rx',msg.topic):
            print(j)
            collection=self.db.uartrx
        else:
            print(f"Unrecognized topic: {msg.topic}")
            collection=self.db.other
        result=collection.insert_one(j)
        print(f"{msg.topic} inserted into {collection.name}")

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

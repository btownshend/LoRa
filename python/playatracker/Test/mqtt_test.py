import json

import paho.mqtt.client as mqtt

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # client.subscribe("$SYS/#")
    client.subscribe("#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    #tv.item(msg.topic,values=(msg.topic, msg.payload))
    value=json.dumps(json.loads(msg.payload),indent=1)
    if tv.exists(msg.topic):
        tv.set(msg.topic,column=1,value=value)
    else:
        tv.insert(parent='', index=tk.END, iid=msg.topic, text='', values=(msg.topic,value))

    # for r in vars:
    #     if r[0].get()==msg.topic:
    #         r[1].set(msg.payload)
    #         return
    # for r in vars:
    #     if r[0].get()=="Empty":
    #         r[0].set(msg.topic)
    #         r[1].set(msg.payload)
    #         return;
    # print('Not enough rows')

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.0.246", 1883, 60)



# importing the module tkinter
import tkinter as tk
from tkinter import ttk

# create main window (parent window)
root = tk.Tk();
root.geometry("800x600")
top=tk.Frame(root)
top.pack();
vars=[[tk.StringVar(value="Empty") for c in range(2)] for r in range(6)]
# lbls=[[tk.Label(top, justify=tk.LEFT, wraplength=800, textvariable=vv) for vv in v] for v in vars]
# for r in range(len(lbls)):
#     for c in range(len(lbls[r])):
#         lbls[r][c].grid(row=r,column=c,sticky=tk.NW)
#
# for r in range(len(lbls)):
#     tk.Grid.rowconfigure(top,r,weight=1)
# for c in range(len(lbls[0])):
#     tk.Grid.columnconfigure(top,c,weight=1)
ttk.Style().configure('Treeview',rowheight=100)
tv = ttk.Treeview(top)
tv['columns']=('Topic', 'Payload')
tv.column('#0', width=0, stretch=tk.NO)
tv.column('Topic', anchor="nw", width=300, stretch=tk.YES)
tv.column('Payload', anchor="nw", width=800, stretch=tk.YES)

tv.heading('#0', text='', anchor=tk.CENTER)
tv.heading('Topic', text='Topic', anchor=tk.CENTER)
tv.heading('Payload', text='Payload', anchor=tk.CENTER)

tv.insert(parent='', index=0, iid=0, text='', values=('Vineet','Alpha'))

tv.pack()


# Start the MQTT handler loop
client.loop_start()

# running the main loop
root.mainloop()
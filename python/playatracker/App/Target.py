# noinspection PyPep8Naming
from datetime import datetime

def parseTime(s):
    # Parse a time of the format "2021-05-24T04:00:12.377589Z"
    d=datetime.strptime(s,"%Y-%m-%dT%H:%M:%S.%fZ")
    return d

class Target:
    def __init__(self, devEUI):
        self.devEUI = devEUI
        self.lastmsg = None
        self.lastloc = None
        self.nupdates = 0

    def update(self, msg):
        # Update state using JSON message
        self.lastmsg = msg
        self.nupdates += 1
        print(f"update {self.devEUI}: lastmsg={self.lastmsg}")
        obj = msg['object']
        if 'latitude' in obj:
            print(f"lastloc={self.lastloc}")
            rxinfo = msg['rxInfo'][0]
            obj = msg['object']
            self.lastloc = obj['latitude'], obj['longitude'], parseTime(rxinfo['time']),

    def getmsg(self):
        return self.lastmsg

    def getfield(self,field):
        return self.lastmsg[field]

    # noinspection PyPep8Naming
    def sendreply(self, dest, target):
        # Build a reply to send to dest, which is tracking target (integer 0..n)
        # Use gateway location if target is 0
        # Otherwise, lookup devEUI
        if target is None:
            lastloc = self.getGatewayLocation()
        else:
            lastloc = target.getLocation()
        print(f"sendreply({dest},{target},{lastloc})")
        # TODO

    def getGatewayLocation(self):
        # Get gateway location using rxInfo
        if self.lastmsg is None:
            return None, None, None,
        rxinfo = self.lastmsg['rxInfo'][0]
        loc = rxinfo['location']
        return loc['latitude'], loc['longitude'], parseTime(rxinfo['time']),

    def getLocation(self):
        # Retrieve location tuple (lat,long,time of last fix)
        if self.lastloc is None:
            return None, None, None,
        else:
            return self.lastloc


targets = {}  # Data on each target (indexed by devEUI)

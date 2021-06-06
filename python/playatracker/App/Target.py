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
        if 'tracking' in obj:
            self.tracking = obj['tracking']

    # noinspection PyPep8Naming
    def sendreply(self):
        # Build a reply to send
        # Use gateway location if target is 1
        # Otherwise, lookup devEUI
        if self.tracking == 0:
            # Compass tracking, nothing to send
            return

        # Find unit
        sendloc = None
        print(f"targets={targets}")
        for t in targets.values():
            if t.unitNumber == self.tracking:
                if sendloc is not None:
                    print(f"*** Multiple devices with unit {self.tracking}")
                sendloc = t.getLocation()
        if sendloc is None:
            if self.tracking != 7:
                print(f"** Warning: unit {self.tracking} not found - sending GW location")
            sendloc = self.getGatewayLocation()
        print(f"sendreply({self.devEUI},{self.tracking},{sendloc})")
        # Build payload: tracking(1 byte), lat*10000 (3bytes), long*10000(3 bytes), fix seconds( 2 bytes)
        data = bytearray()
        data.extend(self.tracking.to_bytes(1, 'big'))
        latval = round(sendloc[0] * 10000)
        data.extend(latval.to_bytes(3, byteorder='big', signed=True))
        longval = round(sendloc[1] * 10000)
        data.extend(longval.to_bytes(3, byteorder='big', signed=True))
        age = round((datetime.utcnow() - sendloc[2]).total_seconds())
        if age > 65535:
            age = 65535
        data.extend(age.to_bytes(2, byteorder='big'))
        print(f"data={data}")
        self.tracker.sendmessage(self.devEUI, 1, data)

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

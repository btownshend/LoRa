# noinspection PyPep8Naming
from datetime import datetime


def parseTime(s):
    # Parse a time of the format "2021-05-24T04:00:12.377589Z"
    d = datetime.strptime(s, "%Y-%m-%dT%H:%M:%S.%fZ")
    return d


class Target:
    def __init__(self, pt, devEUI):
        self.tracker = pt
        self.devEUI = devEUI
        self.lastmsgtime = None
        self.lastloc = None
        self.nupdates = 0
        self.tracking = 0
        self.rxInfo = None
        self.txInfo = None
        self.fCnt = 0
        self.deviceName = "?"
        self.battery_voltage = 0
        self.replyInterval = 2  # How many frames received between location replies
        self.unitNumber = None

    def dump(self):
        elapsed = (datetime.utcnow() - self.lastmsgtime).total_seconds()
        print(f"{self.unitNumber} {self.deviceName:20s}: lastseen={elapsed:4.0f} frame {self.fCnt:5d} tracking {self.tracking:1d} bat={self.battery_voltage:5.3f}",end="")
        if self.lastloc is not None:
            age = (datetime.utcnow() - self.lastloc[2]).total_seconds()
            print(f" {self.lastloc[0]:8.4f},{self.lastloc[1]:8.4f} age={age:5.0f} ")
        else:
            print("")

    def update(self, msg):
        # Update state using JSON message
        self.nupdates += 1
        print(f"update {self.devEUI}: msg={msg}")
        self.rxInfo = msg['rxInfo'][0]
        self.txInfo = msg['txInfo']
        self.fCnt = msg['fCnt']
        self.deviceName = msg['deviceName']
        self.unitNumber = int(self.deviceName[-1])  # Last digit of deviceName
        # print(f"rxInfo={self.rxInfo}")
        self.lastmsgtime = parseTime(self.rxInfo['time'])
        obj = msg['object']
        if 'latitude' in obj:
            self.lastloc = obj['latitude'], obj['longitude'], self.lastmsgtime,
            print(f"lastloc={self.lastloc}")
        if 'tracking' in obj:
            self.tracking = obj['tracking']
        if 'battery_voltage' in obj:
            self.battery_voltage = obj['battery_voltage']
        if self.lastloc is not None and self.fCnt % self.replyInterval == 0:  # Only queue up when we get a new frame to avoid a backlog
            self.sendreply()

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
        if self.rxInfo is None:
            return None, None, None,
        loc = self.rxInfo['location']
        return loc['latitude'], loc['longitude'], parseTime(self.rxInfo['time']),

    def getLocation(self):
        # Retrieve location tuple (lat,long,time of last fix)
        if self.lastloc is None:
            return None, None, None,
        else:
            return self.lastloc


targets = {}  # Data on each target (indexed by devEUI)

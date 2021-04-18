"""
Example for using the RFM9x Radio with Raspberry Pi.

Learn Guide: https://learn.adafruit.com/lora-and-lorawan-for-raspberry-pi
Author: Brent Rubell for Adafruit Industries
"""
# Import Python System Libraries
import time
# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import the SSD1306 module.
import adafruit_ssd1306
# Import RFM9x
import adafruit_rfm9x
import struct
import sys

myid=int(sys.argv[1])

# Button A
btnA = DigitalInOut(board.D5)
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP

# Button B
btnB = DigitalInOut(board.D6)
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP

# Button C
btnC = DigitalInOut(board.D12)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP

# Create the I2C interface.
i2c = busio.I2C(board.SCL, board.SDA)

# 128x32 OLED Display
reset_pin = DigitalInOut(board.D4)
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)
# Clear the display.
display.fill(0)
display.show()
width = display.width
height = display.height

# Configure LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
rfm9x.tx_power = 23
lastrcvd=0
pcntr=0
myloc=None
rmtloc={}
rmtid=None
rmtcntr=None
rssi=None
lastsend=0
SENDINTERVAL=5   # Time between transmits

# Setup gpsd
import gps
gpssession=gps.gps()
gpssession.stream(gps.WATCH_ENABLE|gps.WATCH_NEWSTYLE)
lastfix=0
GPSINTERVAL=5

def receive():
    global rmtloc, rmtid, rmtcnt, rssi, lastrcvd
    packet = rfm9x.receive()
    if packet is not None:
        if len(packet)==28:
            [rmtid,rmtcntr,lat,lon,rmttime]=struct.unpack('BHddf',packet)
            rmtloc['lat']=lat
            rmtloc['lon']=lon
            rmtloc['last']=rmttime
            rssi=rfm9x.last_rssi
            lastrcvd=time.time()
        else:
            print(f"Bad packet length: {len(packet)}")
        
def getgps():
    # Get GPS Fix if available and update our position
    global lastfix, myloc
    start=time.time()
    while time.time()-lastfix > GPSINTERVAL and time.time()-start < 2:
        report=gpssession.next()
        print(f"report={report}")
        if report['class']=='TPV':
            myloc=report
            print(f"myloc={myloc}")
            lastfix=time.time()
        else:
            print(f"Ignoring report={report['class']}")
    print(f"Time since last fix: {time.time()-lastfix:.1f}")

def updatedisplay():
    lines=[]
    lines.append(f"ID:{rmtid},#{rmtcntr},{rssi}dBm,{time.time()-lastrcvd:.1f}s")
    if myloc is not None and 'lat' in myloc:
        lines.append(f"{myid}: {myloc['lat']:.6f},{myloc['lon']:.6f},{time.time()-lastfix:.1f}")
    if rmtloc is not None and 'lat' in rmtloc:
        lines.append(f"{rmtid}: {rmtloc['lat']:.6f},{rmtloc['lon']:.6f}")

    vpos=0
    for l in lines:
        print(l)
        display.text(l,0,vpos,1)
        vpos+=10
    display.show()
    
def send():
    # Send any messages as needed
    global lastsend, myid, myloc, pcntr
    if time.time()-lastsend<SENDINTERVAL:
        return
    if myloc is None:
        packet=struct.pack('BHddf',myid,pcntr,0.0,0.0,0.0)
    else:
        packet=struct.pack('BHddf',myid,pcntr,myloc['lat'],myloc['lon'],time.time()-lastfix)
    rfm9x.send(packet)
    pcntr+=1
    lastsend=time.time()
    
while True:
    receive()
    getgps()
    send()
    updatedisplay()
    time.sleep(1)

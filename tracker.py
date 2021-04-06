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
packet_text = None
lastrcvd=None
pcntr=0
myloc=None
rmtloc=None
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
    global packet_text, rssi, lastrcvd
    packet = rfm9x.receive()
    if packet is not None:
        # Display the packet text and rssi
        display.fill(0)
        prev_packet = packet
        packet_text = str(prev_packet, "utf-8")
        rssi=rfm9x.last_rssi
        lastrcvd=time.time()
        
def getgps():
    # Get GPS Fix if available and update our position
    global lastfix, myloc
    while time.time()-lastfix > GPSINTERVAL:
        report=gpssession.next()
        if report['class']=='TPV':
            myloc=report
            print(f"myloc={myloc}")
            lastfix=time.time()
        else:
            print(f"Ignoring report={report}")

def updatedisplay():
    display.fill(0)
    lines=[]
    if packet_text is not None:
        lines.append("RX: %s "%packet_text)
        lines.append("RSSI: %.0f   Last: %.0f s"%(rssi,time.time()-lastrcvd),)
    if myloc is not None:
        lines.append("Me: %.6f,%.6f"%(myloc['lat'],myloc['lon']))
    if rmtloc is not None:
        lines.append("Rmt: %.6f,%.6f"%(rmtloc['lat'],rmtloc['lon']))

    vpos=0
    for l in lines:
        print(l)
        display.text(l,0,vpos,1)
        vpos+=10
    display.show()
    
def send():
    # Send any messages as needed
    global lastsend, pcntr
    if time.time()-lastsend<SENDINTERVAL:
        return
    if myloc is not None:
        msg=f"{pcntr} {myloc['lat']},{myloc['lon']}"
    else:
        msg=f"{pcntr} No fix"
    packet=bytes(msg,"utf-8")
    rfm9x.send(packet)
    pcntr+=1
    lastsend=time.time()
    
while True:
    receive()
    getgps()
    send()
    updatedisplay()
    time.sleep(1)

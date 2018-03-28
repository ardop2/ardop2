import time
from serialpacket import SerialPacket

angles = [0, 0, 0]

try:
    dev = SerialPacket('/dev/ttyACM0', 9600)
    dev.data = angles
    dev.sendPacket()

except Exception as err:
    print err
dev.close()

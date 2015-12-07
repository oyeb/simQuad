'''
xbee pinouts:

1.Vcc
2.DOUT (Tx)
3.DIN (Rx)
10.GND
'''
import serial, io

xbee = serial.Serial('/dev/ttyUSB0', 57600, timeout=None)

print('\nconnected')
print("flushing %d bytes"%xbee.inWaiting())
xbee.flushInput()

xbee.write("+++")
print(xbee.read(2))
xbee.write("atmy\r")
myid = xbee.read(5)
xbee.write("atid\r")
panid = xbee.read(5)
xbee.write("atdl\r")
destid = xbee.read(5)

print("myid: %d"%int(myid))
print("panid: %d"%int(panid))
print("destid: %d"%int(destid))
print("Allowing xbee to listen...\n")

xbee.close()
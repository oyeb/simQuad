'''
xbee pinouts:

1.Vcc = 3.3V
2.DOUT (Tx of XBee) connect to RX of arduino
3.DIN (Rx of Xbee) connect to TX of...
10.GND
'''
import serial, random

class radio:

  def __init__(self, port, baudrate):
    assert(baudrate == 115200)
    self.interface = serial.Serial(port, baudrate, timeout=None)
    print('Connected')
    self.interface.flushInput()

  def diagnose(self):
    self.interface.flushInput()
    self.interface.write("+++")
    print(self.interface.read(2))
    self.interface.write("atmy\r")
    myid = int(self.interface.read(5))
    print("myid : %d"%myid)
  
  def notify(self):
    code = random.randint(0, 255)
    self.write(chr(code).encode('latin1'))
    self.interface.flushInput()
    echo = self.readn(1)
    #while echo != (code+1)%256:
    #  echo = self.readn(1)
    print('/-.-\ hand shook. s:%s(%d) r:%s(%d)'%(chr(code), code, echo, ord(echo)))

  def write(self, tx_str):
    self.interface.write(tx_str)

  def readn(self, n):
    return self.interface.read(n)

  def powerdown(self):
    self.interface.close()

if __name__ == '__main__':
  xbee = radio('/dev/ttyUSB0', 57600)
  xbee.diagnose()
  xbee.notify()
  while True:
    print(xbee.readn(1))
  xbee.notify()
  xbee.powerdown()
'''
xbee pinouts:

1.Vcc = 3.3V
2.DOUT (Tx of XBee) connect to RX of arduino
3.DIN (Rx of Xbee) connect to TX of...
10.GND
'''
import serial, random

class radio:
  """
  Provides interface to the serial port or XBee
  Usage in both cases is almost the same.
  On manjaro(Arch linux) I just have to change the `portID`
  See the arg-parser in gs-control.py to get a hint on usage.
  """
  def __init__(self, port, baudrate):
    assert(baudrate == 115200)
    self.interface = serial.Serial(port, baudrate, timeout=None)
    print('Connected')
    self.interface.flushInput()

  def diagnose(self):
    """
    Checks if XBee is working
    """
    self.interface.flushInput()
    self.interface.write("+++")
    print(self.interface.read(2))
    self.interface.write("atmy\r")
    myid = int(self.interface.read(5))
    print("myid : %d"%myid)
    assert(myid == 1000)
  
  def notify(self):
    """
    Initiates a handshake. Can be used to send a command,
    since urrently just a random byte is sent.
    """
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
  print("Make sure XBee Baudrate is 115200!!")
  xbee = radio('/dev/ttyUSB0', 115200)
  xbee.diagnose()
  xbee.notify()
  while True:
    print(xbee.readn(1))
  xbee.notify()
  xbee.powerdown()
'''
xbee pinouts:

1.Vcc = 3.3V
2.DOUT (Tx of XBee) connect to RX of arduino
3.DIN (Rx of Xbee) connect to TX of...
10.GND

l to r vcc gnd scl sda dash dash ad0 int
'''
import serial, random

def longToBytes(long_number):
  """
  converts long_number into list of <8-bit> bytes
  Result Format:
  [hi - mid - ... - lo]
  """
  res = []
  while long_number > 0:
    res.insert(0, long_number % 256)
    long_number = long_number // 256
  return res

def bytesToLong(byte_list):
  """
  converts byte_list into a long long_number
  Expected Format:
  [hi - mid - ... - lo]
  """
  res = 0
  for bb in byte_list:
    res = res << 8
    res += bb
  return res

class radio:
  """
  Provides interface to the serial port or XBee
  Usage in both cases is almost the same.
  On manjaro(Arch linux) I just have to change the `portID`
  See the arg-parser in gs-control.py to get a hint on usage.
  """
  def __init__(self, port, baudrate):
    assert(baudrate == 19200)
    self.interface = serial.Serial(port, baudrate, timeout=None)
    print('Connected')
    self.interface.flushInput()

  def diagnose(self):
    """
    Checks if XBee is working
    """
    self.interface.flushInput()
    self.interface.write(b"+++")
    print(self.interface.read(2))
    self.interface.write(b"atmy\r")
    myid = int(self.interface.read(5).decode('utf8'))
    print("myid : %d"%myid)
    assert(myid == 1000)
  
  def notify(self):
    """
    Initiates a handshake. Can be used to send a command,
    since urrently just a random byte is sent.
    """
    code = random.randint(0, 255)
    self.write(code)
    self.interface.flushInput()
    echo = self.readn(1)
    #while echo != (code+1)%256:
    #  echo = self.readn(1)
    print('/-.-\ hand shook. s:%s(%d) r:%s(%d)'%(chr(code), code, echo, ord(echo)))

  def write(self, tx_obj):
    if isinstance(tx_obj, (int)):
      self.write_byte_list( longToBytes(tx_obj) )
    elif isinstance(tx_obj, str):
      self.interface.write(tx_obj)
    else:
      print("at_talk.write() expects int, long_int, str objects only")
      raise RuntimeError

  def write_byte_list(self, tx_byte_list):
    """
    sends multiple bytes, expects a list of bytes.
    Use longToBytes() if needed
    """
    for bb in tx_byte_list:
      self._write_single_byte(bb)


  def _write_single_byte(self, tx_byte):
    """
    sends only one byte, expects an 8-bit integer
    """
    self.interface.write(chr(tx_byte).encode('latin1'))

  def readn(self, n):
    return self.interface.read(n)

  def powerdown(self):
    self.interface.close()

if __name__ == '__main__':
  print("Make sure XBee Baudrate is 115200!!")
  print("long_number -> bytes demo")
  x = 1000
  print(x, "==", longToBytes(x))

  print('XBee demo')
  xbee = radio('/dev/ttyUSB0', 19200)
  xbee.diagnose()
  xbee.notify()
  while True:
    print(xbee.readn(1))
  xbee.notify()
  xbee.powerdown()

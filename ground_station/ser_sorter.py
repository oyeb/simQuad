import attitude, time

class Sorter:
  """
  uses comms.mode to judge packet length, strips packet and pushes the correct parts into the
  correct queue.
  Manages only recieved data.
  """
  def __init__(self, ns_comms, ns_cfg, ns_qstate, intf):
    self.comms = ns_comms
    self.config = ns_cfg
    self.QuadState = ns_qstate
    self.arduino = intf

  def start(self):
    """
    ns_cfg.comms_active is True only when gs-control wants sorter to work!
    puts most recent packet into the correct namespace member
    """
    up = time.time()
    seq = 0
    while self.config.comms_active:
      if self.comms.mode == 'att_est':
        try:
          num = self.arduino.readn(8)
          if self.config.this_is_v2:
            num = [ord(x) for x in num]
        except:
          print('Serial error!')
          raise RuntimeError
        self.comms.quat_packet = num
        attitude.estimate(num, self.QuadState)
      
      # Another experimental mode of communication
      elif self.comms.mode == 'pings':
        try:
          msg = self.arduino.readn(8)
          print(time.time() - up, msg.decode('utf8'))
          seq += 1
        except:
          print('Serial error!')
          raise RuntimeError
        if (time.time() - up > 1000 * 20):
          print("probably lost %d, re-sending..." % seq)
          self.arduino.write(seq)

      #else: # other modes of comms.


  def end(self):
    self.config.comms_active = False
    self.comms.mode = None
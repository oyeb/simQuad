import attitude

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
      #else: # other modes of comms.


  def end(self):
    self.config.comms_active = False
    self.comms.mode = None
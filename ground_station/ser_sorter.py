class Sorter:
  """
  uses comms.mode to judge packet length, strips packet and pushes the correct parts into the
  correct queue.
  Manages only recieved data.
  """
  def __init__(self, ns_comms, ns_cfg, queue_gs, queue_attest, intf):
    self.comms = ns_comms
    self.q_gs = queue_gs
    self.q_attest = queue_attest
    self.config = ns_cfg
    self.arduino = intf

  def start(self):
    """
    ns_cfg.comms_active is True only when gs-control wants sorter to work!
    """
    while self.config.comms_active:
      if self.comms.mode == 'att_est':
        try:
          num = self.arduino.readn(12)
          num = [ord(x) for x in num]
        except:
          print 'Serial error!'
          raise RuntimeError
        self.q_attest.put(num)
        self.comms.packet = num

  def end(self):
    self.config.comms_active = False
    self.comms.mode = None
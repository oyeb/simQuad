import numpy as np

def variance(readings):
  sum1 = np.zeros(3)
  sum2 = np.zeros(3)
  i=0
  while i<10:
    sum1 += readings[i]
    sum2 += readings[i]*readings[i]
    i+=1
  num = (sum1*sum1)/10
  sd  = (sum2-num)/9
  var = np.sqrt(sd)
  return var

class state_estimator:

  def __init__(self, ns_state, ns_vis, ns_cfg, queue_attest):
    self.QuadState = ns_state
    self.config = ns_cfg
    self.mpl_vis = ns_vis
    self.data = [0]*6
    self.q_attest = queue_attest

  def kalman_loop(self):
    """
    packet: list of 12 raw bytes {convert to signed ints, grouped into (gyro3, accel3)}
    """
    #fout = open('flog.log', 'w')
    while self.config.comms_active:
      try:
        packet = self.q_attest.get(True, .008)
      except:
        continue
      _ind=0
      for i in range(0,12, 2):
        self.data[_ind] = (packet[i]<<8)|packet[i+1]
        if self.data[_ind] & 0x8000:
          self.data[_ind] = self.data[_ind] - 0x10000
        _ind += 1
      accel3 = np.array([float(self.data[0])/self.config.a_scale, float(self.data[1])/self.config.a_scale, float(self.data[2])/self.config.a_scale])
      gyro3  = np.array([float(self.data[3])/self.config.g_scale, float(self.data[4])/self.config.g_scale, float(self.data[5])/self.config.g_scale])
      # converted packet -> data -> gyro3, accel3
      #fout.write("%s %s\n"%(accel3, gyro3))
      print (accel3, gyro3)
    
    #fout.close()
    print('Estimation has paused')
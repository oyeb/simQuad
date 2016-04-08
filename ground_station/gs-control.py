"""
Launches the GroundStation.
Initialises all the submodules. __init__.py should do this instead.
Utilises:

* `at_talk`
Handles communications, supports wired/wireless as well as all commands and communication modes.
* `ser_sorter`
Has complete control over the receiving end of the Serial Communications. Decides where a "packet" deserves to go (via Queues or Namspaces). `at_talk` and `ser_sorter` complement each other.
	+ `attitude`
	Performs (if any) computation to maintain a copy os the quadcopter state in the GroundStation.
* `visual`
Uses VisPy and quadcopter state (computed by `attitude`) to visualise the quadcopter and displays other metrics.
	+ `testing`
	Utility finctions and classes for mesh generation etc.

`multiprocessing.Manager.Namespace`s are used to share global information with sub-modules. All synch-ing is done internally!
`multiprocessing.Queue` is used for communication between modules if necessary.

@devs:

+ Please read about
	* context managers [here](https://pymotw.com/2/multiprocessing/communication.html#shared-namespaces)
	* Queues [here](https://docs.python.org/3/library/multiprocessing.html#exchanging-objects-between-processes)
	* `multiprocessing` guidelines [here](https://docs.python.org/3/library/multiprocessing.html#programming-guidelines)
+ Updation of namespace list or dict members is tricky.
	* You must make a local copy (in the function), change it, then reassign it to the <namespace>.<list-var>.
"""
import multiprocessing
import ser_sorter, at_talk, time, visual
import sys, numpy as np
from vispy.util.quaternion import Quaternion

# Serial objects are not pickle-able, hence they cannot be a part of a Manager.Namespace and thus serial port is a global.
arduino = at_talk.radio('/dev/ttyUSB0', 19200)

# There are multiple namespaces for flexibility
# {ns_comms, ns_qstate, ns_vis, ns_cfg}
mgr = multiprocessing.Manager()
ns_comms = mgr.Namespace()
ns_comms.name = "Communications:\n\tMode that ser_sorter operates in(comms_mode)\n\tRecieved packet from mpu(quat_packet)\n\tRecieved packet for GS(gs_packet)"
ns_comms.quat_packet = None
ns_comms.gs_packet = None
ns_comms.mode = 'att_est'

ns_qstate = mgr.Namespace()
ns_qstate.name = "QuadState:\n\tAttitude Quaternion(heading)"
ns_qstate.heading = Quaternion()

ns_cfg = mgr.Namespace()
ns_cfg.a_scale = 16384.0     # Accel 2g
ns_cfg.g_scale = 16.384      # Gyro  2000 dps
ns_cfg.TIME_INTERVAL = 0.02 # ms
ns_cfg.comms_active = False
ns_cfg.this_is_v2 = sys.version_info[0] == 2

# Queues

vis_canvas = visual.Canvas(800, 600, ns_qstate)
sorter = ser_sorter.Sorter(ns_comms, ns_cfg, ns_qstate, arduino)

ns_cfg.comms_active = True
p_sorter = multiprocessing.Process(target=sorter.start)
p_sorter.daemon = True
vis_canvas.show()

time.sleep(2.5)
arduino.notify()
p_sorter.start()

ch = ""
while ch != "quit":
  if ns_cfg.this_is_v2:
    ch = raw_input('> ')
  else:
    ch = input('> ')

vis_canvas.close()
sorter.end() #cascades to the estimator
arduino.notify()
p_sorter.join()
print('sort:%d'%(p_sorter.pid))


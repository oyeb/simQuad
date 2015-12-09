import multiprocessing, at_talk, attitude, Queue, ser_sorter, time

mgr = multiprocessing.Manager()
ns_comms = mgr.Namespace()
ns_comms.name = "Communications:\n\tat_talk.radio(arduino)\n\tRecieved packet(packet)"
arduino = at_talk.radio('/dev/ttyACM0', 57600)
ns_comms.packet = None
ns_comms.mode = 'att_est'

ns_qstate = mgr.Namespace()
ns_qstate.name = "QuadState:\n\tnothing yet!"

ns_vis = mgr.Namespace()
ns_vis.name = "MatPlotLib:\n\tnothing yet!"

ns_cfg = mgr.Namespace()
ns_cfg.a_scale = 16384.0 #Accel 2g
ns_cfg.g_scale = 65.5 #Gyro 500
ns_cfg.TIME_INTERVAL = 0.005
ns_cfg.comms_active = False

Q_GS = None
Q_AttEst = multiprocessing.Queue()

sorter = ser_sorter.Sorter(ns_comms, ns_cfg, Q_GS, Q_AttEst, arduino)
estimator = attitude.state_estimator(ns_qstate, ns_vis, ns_cfg, Q_AttEst)

ns_cfg.comms_active = True
p_estimator = multiprocessing.Process(target=estimator.kalman_loop)
p_estimator.daemon = False
p_sorter = multiprocessing.Process(target=sorter.start)
p_sorter.daemon = True

time.sleep(2.5)
arduino.notify()
p_estimator.start()
p_sorter.start()

ch = ""
while ch != "quit":
  ch = raw_input('> ')

sorter.end() #cascades to the estimator
arduino.notify()
p_sorter.join()
p_estimator.join()
print('est:%d\nsort:%d'%(p_estimator.pid, p_sorter.pid))


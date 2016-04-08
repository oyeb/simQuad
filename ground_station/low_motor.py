import at_talk
"""
Use `python -m serial.tools.list_ports` to find which port the XBee is using
ttySX, where X={0,1,2...} cannot be used by an XBee, exclude them from consideration.

This is a very rudimentry motor control that has been tested with `arduino/esc_control/manual_motors.ino`
All motors spin at the same speed.
"""
arduino = at_talk.radio('/dev/ttyUSB0', 115200)

pw = 620
while True:
	ch = input("> ")
	if ch.isalpha():
		if ch == 'a':
			pw += 20
		if ch == 'd':
			pw -= 20
		if ch == 's':
			pw = 1040
		if ch == 'm':
			pw = 1950
		#if ch == 'x':
	#		pw = 1350
		if ch == 'q':
			arduino.write( 1040 )
			print("Bye!")
			arduino.powerdown()
			break
		arduino.write( pw )
		recv = []
		for i in [1,2]:
			recv.append( ord(arduino.readn(1)) )
		print("Sent:", pw, "Got:", at_talk.bytesToLong(recv))
	elif int(ch) > 1950 or int(ch) < 1040:
		print("Invalid pulse width")
	else:
		pw = int(ch)
		arduino.write( pw )
		recv = []
		for i in [1,2]:
			recv.append( ord(arduino.readn(1)) )
		print("Sent:", pw, "Got:", at_talk.bytesToLong(recv))
		
	
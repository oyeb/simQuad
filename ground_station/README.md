Ground-Station
==============

Launches the GroundStation. GroundStation can display live data on terminal as well as take input from `stdin` *(though you can't see it when you type it in)*
Initialises all the submodules. `__init__.py` should do this instead.

Usage
-----
`python gs-control.py -h` shows all arg. options

*For WIRED-MODE with a USB cable,*
> Default **portID is `/dev/ttyACM0` in this mode**
```bash
python gs-control.py [--port <PORT_ID>]
```

*For WIRELESS-MODE, with an XBee radio,*
> Default **portID is `/dev/ttyUSB0`**
```bash
python gs-control.py -wl [--port <PORT_ID>]
```

Utilises
--------
* `rxControl`
Has complete control over the receiving end of the Serial Communications. Decides where a "packet" deserves to go (via Queues or Namspaces).
	+ `attitude`
	Performs (if any) computation to maintain a copy os the quadcopter state in the GroundStation.
* `at_talk`
Handles communications, supports wired/wireless as well as all commands and communication modes. Low level module, used directly only by `gs-control` or `rxControl`.
* `visual`
Uses VisPy and quadcopter state (computed by `attitude`) to visualise the quadcopter and displays other metrics.
	+ `vis_util`
	Utility finctions and classes for mesh generation etc.

`multiprocessing.Manager.Namespace` instances are used to share global information with sub-modules. All synch-ing is done internally!
`multiprocessing.Queue` is used for communication between modules if necessary.

@devs
=====

Please read about

* context managers [here](https://pymotw.com/2/multiprocessing/communication.html#shared-namespaces)
* Queues [here](https://docs.python.org/3/library/multiprocessing.html#exchanging-objects-between-processes)
* `multiprocessing` guidelines [here](https://docs.python.org/3/library/multiprocessing.html#programming-guidelines)

Updation of namespace list or dict members is tricky.

* You must make a local copy (in the function), change it, then reassign it to the `{namespace}.{list-var}`.

Computation of RPY
------------------

Display [Tait-Bryan angles](https://en.wikipedia.org/wiki/Euler_angles#Tait.E2.80.93Bryan_angles) by computing them from the quaternion using [this](https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_.E2.86.94_Quaternion) transformation :
$$ roll\ =\ atan2( 2(wx + yz), 1-2(x^2+y^2)) $$
$$ pitch\ =\ arcsin( 2(wy - xz)) $$
$$ yaw\ =\ atan2( 2(wz + xy), 1 - 2(y^2 + z^2)) $$
*scroll down to* **Quaternion → Euler angles (`z-y’-x″` intrinsic)**

todo
----

Display the RPY on the screen, not terminal.
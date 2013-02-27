import sys, time, serial, socket
import subprocess
from threading import Thread

# From http://stackoverflow.com/questions/1093598/pyserial-how-to-read-last-line-sent-from-serial-device
def measure(r, ser):
    buffer = ''

    while True:
        buffer += ser.read(ser.inWaiting())
        if '\n' in buffer:
            lines = buffer.split('\n') # guaranteed to have at least 2 entries
            r.last_serial_input = lines[-2]
            buffer = lines[-1]


class Robbie:
    def __init__(self, ser):
        self.vl = 0.0
        self.vr = 0.0
        self.heading = 0.0

        self.ser = ser
        self._last_serial_input= ''
        self.t = Thread(target = measure, args=(self, ser))

        # TODO: twiddle?
        self.uP = 5.0
        self.uD = 5.0
        self.uI = 5.0

    #    self.t.start()

    def writeCommand(self, vl, vr, longTurn):
        print("%d, %d, %d" % (vl, vr, longTurn))
        self.ser.write(chr(vl+128))
        self.ser.write(chr(vr+128))
        self.ser.write(chr(longTurn+128))

    def turn(self, angle, motors_func):
        prev_error = self.heading
        int_term = 0
        dt = time.time()

        error = 1
        while error > 0.1:
            dt = time.time() - dt
            error = angle - self.heading
            diff_term = error - prev_error / dt
            int_term = (prev_error + error) * dt / 2.0

            control = (self.uP * error) + (self.uD * diff_term) + (self.uI * int_term)
            motors_func(control)
            prev_error = error

    @property
    def vl(self):
        """Extract left wheel velocity from arduino output"""
        return float(self.last_serial_input.split(' ')[0])

    @property
    def vr(self):
        """Extract right wheel velocity from arduino output"""
        return float(self.last_serial_input.split(' ')[1])

    @property
    def heading(self):
        """Extract heading from arduino output"""
        return float(self.last_serial_input.split(' ')[2])


    def inplaceTurn(self, angle):
        self.turn(self.heading + angle, lambda c: self.writeCommand(c, -c, 0))

    def longTurn(self, angle, speed):
        self.turn(self.heading + angle, lambda c: self.writeCommand(speed, speed, c))

    def moveStraight(self, speed):
        print("moveStraight: %d" % speed)
        self.writeCommand(speed, speed, 0)

    def stop(self):
        """ Very useful for avoiding fines for punching holes on the walls """
        self.writeCommand(0, 0, 0)


ser = serial.Serial('/dev/ttyACM0', 9600)
rob = Robbie(ser)
hasConnection = False

while 1:
    HOST = "127.0.0.1"
    PORT = 12345

    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    serversocket.bind((HOST, PORT))
    serversocket.listen(1)

    print("Ready to serve")
    (clientsocket, address) = serversocket.accept()

    print address

    hasClientConnection = True    # is connected to a client

    while hasClientConnection:
        ping = subprocess.Popen(["ping", "-c", "1", "-w", "1", address[0]], shell=False)
    	ping.wait()

    	if ping.returncode != 0:
            print("Lost connection to ", address)
	    hasClientConnection = False
	    command = 'stop\n'
         #   rob.stop()
         #   sys.exit()
	else:
            command = clientsocket.recv(1000)
            print("got command " + command)

        if command == '':
	    rob.stop()
	    hasClientConnecion = False
	    command = 'stop\n'
            # raise RuntimeError("socket connection broken")

    	inp = command.split()

    	if inp[0] == "stop":
            rob.stop()
    	elif inp[0] == "ahead":
            speed = int(inp[1])
            rob.moveStraight(speed)
    	elif inp[0] == "turn":
            speed = int(inp[1])
            rob.writeCommand(speed, -speed, 0)
    	else:
            print "bark bark!"
	    rob.stop()
            #sys.exit()

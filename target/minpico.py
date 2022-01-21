"""
A MIN monitor function for a device with a MicroPython API to the MIN transport layer.

This code is written for the Canis Labs CANPico firmware, which has a MicroPython MIN API
and uses a second (virtual) USB serial port for MIN operations (the first serial port is used for REPL).
"""
from rp2 import MIN
import machine
import binascii


class MINMonitor:
    """
    Base class that handles MIN messages from a host. Can be extended to handle application-specific
    commands (the CANPico CAN API is designed for this: many of the calls can pack or unpack bytes that
    can be carried via MIN)
    """
    PING = 1  # Send back the frame
    IDENTIFY = 2  # Identify this hardware
    STOP = 3  # Stop running the monitor
    PRINT = 4  # Debug print to the REPL console
    EXCEPTION = 10  # A command failed with an exception; report the exception to the host

    def __init__(self, family=b'Pico@', unique_id=machine.unique_id()):
        self.led = machine.Pin(25, machine.Pin.OUT)
        self.m = MIN()
        self.stop = False
        self.family = family
        self.unique_id = unique_id
        self.handlers = {self.PING: self.ping,
                         self.IDENTIFY: self.identify,
                         self.STOP: self.stop,
                         self.PRINT: self.print}

    def loop(self):
        # Receive messages, drive the MIN state machine
        msgs = self.m.recv()
        # Pass incoming messages to handlers
        if msgs:
            for min_id, min_payload in msgs:
                if min_id in self.handlers:
                    try:
                        reply = self.handlers[min_id](min_payload)
                        if reply is not None:
                            self.m.send_frame(reply[0], reply[1])
                    except Exception as e:
                        self.m.send_frame(self.EXCEPTION, bytes(str(e), 'ascii'))

    def run(self):
        while not self.stop:
            self.loop()

    def ping(self, min_payload):
        self.led.toggle()
        return self.PING, min_payload

    def identify(self, min_payload):
        return self.IDENTIFY, self.family + binascii.hexlify(self.unique_id)

    def stop(self, min_payload):
        self.stop = True
        # Doesn't return a MIN frame because the monitor will stop
        return None

    def print(self, min_payload):
        print("PRINT: {}".format(min_payload))
        return self.PRINT, b'ok'

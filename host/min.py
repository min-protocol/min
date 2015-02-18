import queue
import serial
import threading
import time
import argparse
import msvcrt           # Windows-only; on Linux use getch: https://pypi.python.org/pypi/getch

__author__ = 'kentindell'

"""
Microcontroller Interconnect Network (MIN) version 1.0

Python reference implementation and example program

Copyright (c) 2014-2015 JK Energy Ltd.
Licensed under MIT License.
"""


class Frame:
    """
    Class to handle MIN 1.0 frame handling. Constructed after receiving frame data from the serial port and
    also in prelude to sending on the serial port.
    """

    HEADER_BYTE = 0xaa
    STUFF_BYTE = 0x55
    EOF_BYTE = 0x55

    def checksum(self):
        """
        Compute Fletcher's checksum (16-bit version)

        sum1 and sum2 are 16 bit integers so must be clipped back to this range

        Return a list of high byte then low byte (big-endian order on the wire)
        """
        sum1 = 0xff
        sum2 = 0xff

        checksummed_data = [self.frame_id, self.get_control()] + self.payload

        for b in checksummed_data:
            sum1 += b
            sum1 &= 0xffff  # Results wrapped at 16 bits
            sum2 += sum1
            sum2 &= 0xffff

        sum1 = (sum1 & 0x00ff) + (sum1 >> 8)
        sum2 = (sum2 & 0x00ff) + (sum2 >> 8)

        checksum = ((sum2 << 8) & 0xffff) | sum1

        high_byte = (checksum & 0xff00) >> 8
        low_byte = checksum & 0x00ff

        return [high_byte, low_byte]

    def __init__(self, serial_handler, frame_id=0, payload=list()):
        """
        id is the MIN frame id

        payload is a list of up to 15 bytes that is the payload of the frame
        """
        self.frame_id = frame_id
        self.payload = payload
        self.stuffed = None
        self.handler = serial_handler
        if len(self.payload) > 15:
            raise Exception("Payload too big")

    def transmit(self):
        """
        Transmit self through the assigned serial handler
        """
        self.handler.send_queue.put(self.get_bytes())

    def __str__(self):
        s = "ID: 0x{0:02x}\n".format(self.frame_id)
        s += "Payload: {0}\n".format(':'.join('{:02x}'.format(i) for i in self.payload))

        return s

    def get_control(self):
        """
        Get the control byte based on the frame properties

        NB: In MIN 1.0 the top four bits must be set to zero (reserved for future)
        """
        tmp = 0
        tmp |= len(self.payload)

        return tmp

    def get_payload(self):
        return self.payload

    def get_bytes(self):
        """
        Get the on-wire byte sequence for the frame, including stuff bytes after every 0xaa 0xaa pair
        """
        raw = [self.frame_id] + [self.get_control()] + self.payload + self.checksum()
        self.stuffed = [self.HEADER_BYTE, self.HEADER_BYTE, self.HEADER_BYTE]

        count = 0

        for i in raw:
            self.stuffed.append(i)
            if i == self.HEADER_BYTE:
                count += 1
                if count == 2:
                    self.stuffed.append(self.STUFF_BYTE)
                    count = 0
            else:
                count = 0

        self.stuffed.append(self.EOF_BYTE)

        return self.stuffed

    def get_length(self):
        return len(self.payload)

    def get_id(self):
        return self.frame_id


class SerialHandler:
    """
    This class handles the serial port and sends frames to the port and receives them from the port.

    It creates two threads, one for writing and one for reading
    """

    # States for receiving a frame
    SOF, ID, CONTROL, PAYLOAD, CHECKSUM_HIGH, CHECKSUM_LOW, EOF = range(7)

    def __init__(self, port, baudrate, received_frame_handler):
        self.header_bytes_seen = 0
        self.frame_id = 0
        self.frame_length = 0
        self.frame_payload = []
        self.frame_checksum_bytes = []
        self.payload_bytes_to_go = 0
        self.frame = None
        self.state = self.SOF
        self.serial = serial.Serial(port=port,
                                    baudrate=baudrate,
                                    timeout=None,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS)

        self.received_frame_handler = received_frame_handler

        # Initialize receiver and sender threads
        self.send_queue = queue.Queue()

        self.receive_thread = threading.Thread(target=self.receiver)
        self.send_thread = threading.Thread(target=self.sender)

        self.receive_thread.daemon = True
        self.send_thread.daemon = True

        self.receive_thread.start()
        self.send_thread.start()

    def receiver(self):
        """
        Receive loop that takes a byte at a time from the serial port and creates a frame
        """
        while True:
            # Read a byte from the serial line (blocking call)
            data = self.serial.read(size=1)
            if args.show_raw:
                print("Data RX on wire: " + '0x{:02x}'.format(data[0]))
            self.build_received_frame(data[0])

    def sender(self):
        """
        Feed the queue into the serial port (blocking on reading the queue and the sending)
        """
        while True:
            frame_data = self.send_queue.get()
            if args.show_raw:
                print("Data TX on wire: %s" % ':'.join('0x{:02x}'.format(i) for i in frame_data))
            self.serial.write(data=frame_data)

    def build_received_frame(self, byte):
        """
        Read bytes in sequence until a frame has been pulled in
        """
        if self.header_bytes_seen == 2:
            self.header_bytes_seen = 0
            if byte == Frame.HEADER_BYTE:
                # If three header bytes in a row, reset state machine and start reading a new frame
                if args.show_raw:
                    print("Header seen")
                self.state = SerialHandler.ID
                return
            # Two in a row: we should see a stuff byte
            if byte != Frame.STUFF_BYTE:
                # Something has gone wrong with the frame, discard and reset
                print("Framing error: Missing stuff byte")
                self.state = SerialHandler.SOF
                return
            else:
                # A stuff byte, discard and carry on receiving on the next byte where we were
                if args.show_raw:
                    print("Stuff byte discarded")
                return

        if byte == Frame.HEADER_BYTE:
            self.header_bytes_seen += 1
        else:
            self.header_bytes_seen = 0

        if self.state == SerialHandler.ID:
            if args.show_raw:
                print("ID byte")
            self.frame_id = byte
            self.state = SerialHandler.CONTROL
        elif self.state == SerialHandler.CONTROL:
            if args.show_raw:
                print("control byte")
            self.frame_length = byte & 0x0f
            if args.show_raw:
                print("control byte %s [length=%d]"
                      % ('0b{:08b}'.format(byte),
                         self.frame_length))
            self.payload_bytes_to_go = self.frame_length
            self.frame_payload = []
            if self.payload_bytes_to_go > 0:
                self.state = SerialHandler.PAYLOAD
            else:
                self.state = SerialHandler.CHECKSUM_HIGH
        elif self.state == SerialHandler.PAYLOAD:
            if args.show_raw:
                print("payload byte")
            self.frame_payload.append(byte)
            self.payload_bytes_to_go -= 1
            if self.payload_bytes_to_go == 0:
                self.state = SerialHandler.CHECKSUM_HIGH
        elif self.state == SerialHandler.CHECKSUM_HIGH:
            if args.show_raw:
                print("checksum high")
            self.frame_checksum_bytes = [byte]
            self.state = SerialHandler.CHECKSUM_LOW
        elif self.state == SerialHandler.CHECKSUM_LOW:
            if args.show_raw:
                print("checksum low")
            self.frame_checksum_bytes.append(byte)
            # Construct the frame object
            self.frame = Frame(serial_handler=self,
                               frame_id=self.frame_id,
                               payload=self.frame_payload)
            checksum_bytes = self.frame.checksum()
            if checksum_bytes != self.frame_checksum_bytes:
                # Checksum failure, drop it and look for a new one
                print("FAILED CHECKSUM")
                self.state = SerialHandler.SOF
            else:
                self.state = SerialHandler.EOF
        elif self.state == SerialHandler.EOF:
            if byte == Frame.EOF_BYTE:
                if args.show_raw:
                    print("EOF, frame passed up")
                    print(self.frame)
                # Frame is well-formed,pass it up for handling
                self.received_frame_handler(frame=self.frame)

            self.state = SerialHandler.SOF


# Decoder MIN network order 16-bit and 32-bit words
def min_decode(data):
    if len(data) == 2:
        # 16-bit big-endian integer
        return (data[0] << 8) | (data[1])
    elif len(data) == 4:
        # 32-bit big-endian integer
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3])


# Encode a 32-bit integer into MIN network order bytes
def min_encode_32(x):
    return [(x & 0xff000000) >> 24, (x & 0x00ff0000) >> 16, (x & 0x0000ff00) >> 8, (x & 0x000000ff)]


# Encode a 16-bit integer into MIN network order bytes
def min_encode_16(x):
    return [(x & 0x0000ff00) >> 8, (x & 0x000000ff)]


# Called when a MIN frame has been received successfully from the serial line
def received_frame(frame):
    message_id = frame.get_id()
    data = frame.get_payload()

    if args.quiet:
        pass
    else:
        if message_id == 0x0e:      # Deadbeef message
            print("RX deadbeef: " + ':'.join('{:02x}'.format(i) for i in data))
        elif message_id == 0x23:            # Environment message
            temperature = -20.0 + (min_decode(data[0:2]) * 0.0625)
            humidity = min_decode(data[2:4]) * 0.64
            print("Environment: temperature={0}C, humidity={1}%".format(temperature, humidity))
        elif message_id == 0x24:            # Motor status message
            status = data[0]
            position = min_decode(data[1:5])
            print("Motor: status={}, position={}".format(status, position))
        elif message_id == 0x02:
            print("Ping received: " + ':'.join('{:02x}'.format(i) for i in data))


# Interactive menu to send frames on a keypress
def interactive_controller():
    print("Interactive MIN controller:")
    print("p        Send ping frame")
    print("m        Send motor request")

    while True:
        ch = msvcrt.getch()
        if ch == b'p':
            f = Frame(controller, frame_id=0x02, payload=[0xca, 0xfe, 0xf0, 0x0d])
            f.transmit()
            print("Ping frame queued")
        elif ch == b'm':
            payload = min_encode_32(1000000) + min_encode_16(5000)
            f = Frame(controller, frame_id=0x36, payload=payload)
            f.transmit()
            print("Motor request sent")
        else:
            pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="MIN controller")

    parser.add_argument('-p', dest='port', default="COM4", type=str, help="Serial port, e.g. COM3")
    parser.add_argument('-listen_only', action='store_true')
    parser.add_argument('-baud', default=9600, type=int)
    parser.add_argument('-show_raw', action='store_true')
    parser.add_argument('-quiet', action='store_true')

    args = parser.parse_args()
    controller = SerialHandler(port=args.port, baudrate=args.baud, received_frame_handler=received_frame)

    if args.listen_only:
        print("Listening only")
    else:
        interactive_controller()

    print("MIN controller running")
    time.sleep(3600.0)

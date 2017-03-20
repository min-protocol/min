"""
Simple example program that sends a MIN frame and displays all received MIN frames.
"""
from struct import unpack
from time import sleep, time

from min import MINTransportSerial, bytes_to_hexstr

# Linux USB serial ports are of the form '/dev/ttyACM*'
# macOS USB serial ports are of the form '/dev/tty.usbmodem*'
# Windows randomly assigns COM ports, depending on the driver for the USB serial chip.
# Genuine FTDI chips tend to end up at the same port between reboots. YMMV.
MIN_PORT = '/dev/tty.usbmodem1421'


def bytes_to_int32(data: bytes, big_endian=True) -> int:
    if len(data) != 4:
        raise ValueError("int32 shoud be exactly 4 bytes")
    if big_endian:
        return unpack('>I', data)[0]
    else:
        return unpack('<I', data)[0]


def wait_for_frames(min_handler: MINTransportSerial):
    while True:
        # The polling will generally block waiting for characters on a timeout
        # How much CPU time this takes depends on the Python serial implementation
        # on the target machine
        frames = min_handler.poll()
        if frames:
            return frames


if __name__ == "__main__":
    # One approach to autodiscovering a port with a MIN device on the other end is to open
    # all the ports on the machine and listen for a heartbeat: the chances of random data
    # appearing as a valid MIN frame with a magic number in the payload are virtually zero.
    min_handler = MINTransportSerial(port=MIN_PORT, debug=False)

    min_id = 0x01
    while True:
        payload = bytes("hello world {}".format(time()), encoding='ascii')
        # Send a frame on the serial line
        min_handler.queue_frame(min_id=min_id, payload=payload)

        # Wait for one or more frames to come back from the serial line and print them out
        for frame in wait_for_frames(min_handler):
            print("Frame received: min ID={}".format(frame.min_id))
            if frame.min_id == min_id + 1:
                print("(In ASCII: '{}')".format(frame.payload))
            elif frame.min_id == 0x33:
                print("(Time = {})".format(bytes_to_int32(frame.payload, big_endian=False)))  # SAMD21 is little-endian
        # Wait a little bit so we don't flood the other end
        sleep(0.5)

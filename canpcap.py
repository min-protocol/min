"""
Simple example program that reads MIN frames that contain captured CAN frames and pushes them to stdout
for tshark or wireshark to use.

To run:

$ python3 canpcap.py -p [port] | tshark -i -

where [port] is the serial port that connects to a MIN device uploading CAN frames. See the function minmon()
in the file canpico.py in the CANHack repository for an example of a simple MicroPython monitor function.
"""
import logging
import struct
from struct import unpack
from time import sleep, time
import sys
import getopt
from min import MINTransportSerial
from binascii import hexlify

# Linux USB serial ports are of the form '/dev/ttyACM*' and the CANPico board
# from Canis Labs runs two serial ports, one for the REPL console and one
# for applications to use; the default CANPico firmware includes MIN support
# on this second port
MIN_PORT = '/dev/ttyACM1'


def wait_for_frames(min_handler: MINTransportSerial):
    while True:
        # The polling will generally block waiting for characters on a timeout
        # How much CPU time this takes depends on the Python serial implementation
        # on the target machine
        frames = min_handler.poll()
        if frames:
            return frames


class CANPCAPNG:
    """
    The file produces:

    - SHB (Section header block)
    - IDB (Interface description block)
    - Multiple EPBs (Extended packet blocks)

    The SHB defines the file size and endianness.

    The IDB defines the link type, which will be passed through into protocol decoders.
        - The option if_tsresol should be set to select nanosecond timestamp resolution (since 1st Jan 1970)

    The EPBs define the CAN frames. The timestamp is 64-bit. The interface ID matches the one in the IBD.
    """

    @staticmethod
    def get_shb() -> bytes:
        return b"".join(struct.pack('>I', i) for i in [0x0a0d0d0a, 28, 0x1a2b3c4d, 0x00010000, 0xffffffff, 0xffffffff, 28])

    @staticmethod
    def get_idb() -> bytes:
        return b"".join(struct.pack('>I', i) for i in [0x00000001, 20, 227 << 16, 0, 20])

    @staticmethod
    def get_epbs(b) -> bytes:
        """
        Returns a block of EPBs from a block of bytes from the CANPico
        MicroPython firmware recv() call. Events are always 19 bytes.

        Format is:

        n+0: flags byte
            bits 1 to 0: event type (0=transmitted, 1=received CAN frame, 2=CAN error, 3=FIFO overflow)
            bit 7: remote frame

        n+1: timestamp (big-endian format)

        If received frame:
            n+5: dlc
            n+6: ID filter hit
            n+7: CAN ID (32-bit big-endian word)
            n+11: payload bytes

        CAN ID 32-bits in the following format:
            bit 29: IDE flag
            bits 28 to 18: ID A
            bits 17 to 0: ID B
        """
        epbs = bytes()

        while len(b) > 0:
            eventb = b[:19]
            b = b[19:]
            logging.debug("len={}".format(len(eventb)))

            if eventb[0] & 0x3 == 1:
                rtr = (eventb[0] >> 7) & 1
                timestamp = struct.unpack('>I', eventb[1:5])[0]
                dlc = eventb[5]
                canid = struct.unpack('>I', eventb[7:11])[0]
                ida = (canid >> 18) & 0x7ff
                idb = canid & 0x3ffff
                ide = (canid >> 29) & 1
                if dlc > 8:
                    l = 8
                else:
                    l = dlc
                data = eventb[11:11 + l]

                logging.debug("dlc={}".format(dlc))
                logging.debug("ide={}".format(ide))
                logging.debug("rtr={}".format(rtr))
                logging.debug("ida={}".format(ida))
                logging.debug("idb={}".format(idb))
                logging.debug("data={}".format(data))
                logging.debug("timestamp={}".format(timestamp))
                logging.debug(hexlify(eventb))

                epbs += CANPCAPNG.get_epb(ida=ida,
                                          idb=idb,
                                          ide=ide,
                                          rtr=rtr,
                                          data=data,
                                          timestamp=timestamp)
        return epbs


    @staticmethod
    def get_epb(ida: int, idb: int, ide: int, rtr: int, data: bytes, timestamp: int, interface_id=0, error_frame=False) -> bytes:
        assert ide in [1, 0]
        assert rtr in [1, 0]
        assert ida < (1 << 11)
        assert idb < (1 << 18)
        assert len(data) <= 8

        if ide:
            canid = ida << 18 | idb | 0x80000000
        else:
            canid = ida
        if rtr:
            canid |= 0x40000000
        if error_frame:
            canid = 0x20000000
            data = bytes([8] * 8)  # Error reports need to be 8 bytes of data
        padded_data = (data + bytes([0] * 8))[:8]

        data_h = struct.unpack('>I', padded_data[:4])[0]
        data_l = struct.unpack('>I', padded_data[4:])[0]

        return b"".join(struct.pack('>I', i) for i in [0x00000006,
                                                       48,
                                                       interface_id,
                                                       timestamp >> 32,
                                                       timestamp & 0xffffffff,
                                                       16,
                                                       16,
                                                       canid,
                                                       len(data) << 24,
                                                       data_h,
                                                       data_l,
                                                       48])


if __name__ == "__main__":
    options = "hp:"

    port = MIN_PORT

    try:
        args, vals = getopt.getopt(sys.argv[1:], options)
        for arg, val in args:
            if arg in ("-h"):
                print ("-p [port]")
                quit()
            elif arg in ("-p"):
                port = val
                print("port={}".format(port))

    except getopt.error as err:
        print (str(err))
        quit()

    logging.basicConfig(level=logging.WARNING)

    min_handler = MINTransportSerial(port=port)
    logging.debug("Polling for MIN frames")


    hdr = CANPCAPNG.get_shb() + CANPCAPNG.get_idb()
    sys.stdout.buffer.write(hdr)

    min_handler.transport_reset()

    while True:
        # Wait for one or more frames to come back from the serial line and print them out
        for frame in wait_for_frames(min_handler):
            if frame.min_id == 1:
                epbs = CANPCAPNG.get_epbs(b=frame.payload)
                sys.stdout.buffer.write(epbs)
                sys.stdout.buffer.flush()


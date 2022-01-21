"""
Soak test for transport layer
"""
from logging import DEBUG, FileHandler
from random import randrange

from time import time
import sys
import struct

from min import MINTransportSerial, min_logger


MIN_PORT = '/dev/ttyACM1'
WSHEND = '<I'


def bytes_to_int32(data: bytes, big_endian=True) -> int:
    if len(data) != 4:
        raise ValueError("int32 shoud be exactly 4 bytes")
    if big_endian:
        return struct.unpack('>I', data)[0]
    else:
        return struct.unpack('<I', data)[0]


def wait_for_frames(min_handler: MINTransportSerial, timeout=3.0):
    start = time()
    while True:
        frames = min_handler.poll()
        if frames:
            return frames


class CANPCAPNG:
    """
    Generates a binary capture file in pcapng format. This is documented:

    https://tools.ietf.org/id/draft-tuexen-opsawg-pcapng-02.txt

    The file produces:

    - SHB (Section header block)
    - IDB (Interface description block)
    - Multiple EPBs (Extended packet blocks)

    The SHB defines the file size and endianness.

    The IDB defines the link type, which will be passed through into protocol decoders.
        - The option if_tsresol should be set to select nanosecond timestamp resolution (since 1st Jan 1970)

    The EPBs define the CAN frames. The timestamp is 64-bit. The interface ID matches the one in the IBD.

    The link type is 227 (see http://www.tcpdump.org/linktypes.html), and the packet format is:

        +---------------------------+
        |      CAN ID and flags     |
        |         (4 Octets)        |
        +---------------------------+
        |    Frame payload length   |
        |         (1 Octet)         |
        +---------------------------+
        |          Padding          |
        |         (1 Octet)         |
        +---------------------------+
        |      Reserved/Padding     |
        |         (1 Octet)         |
        +---------------------------+
        |      Reserved/Padding     |
        |         (1 Octet)         |
        +---------------------------+
        |           Payload         |
        .                           .
        .                           .
        .                           .

        Description

        The field containing the CAN ID and flags is in network byte order (big-endian). The bottom 29 bits contain the CAN ID of the frame. The remaining bits are:

            0x20000000 - set if the frame is an error message rather than a data frame.
            0x40000000 - set if the frame is a remote transmission request frame.
            0x80000000 - set if the frame is an extended 29-bit frame rather than a standard 11-bit frame. frame.

        The SocketCAN dissector in wireshark might be able to handle a larger payload than 8 bytes.

        # TODO create a new link layer definition with a LUA dissector that can handle more information
    """

    @staticmethod
    def get_shb() -> bytes:
        # return b"".join(struct.pack(WSHEND, i) for i in [0x0a0d0d0a, 28, 0x1a2b3c4d, 0x00010000, 0xffffffff, 0xffffffff, 28])
        return b"".join(struct.pack(WSHEND, i) for i in [0x0a0d0d0a, 28, 0x1a2b3c4d, 0x00000001, 0xffffffff, 0xffffffff, 28])

    @staticmethod
    def get_idb() -> bytes:
        # Link type is SocketCAN, 227 (see http://www.tcpdump.org/linktypes.html)
        # TODO set timestamps to be in nanoseconds, rather than microseconds, by adding the if_tsresol option
        # TODO option type: 9
        # TODO option length: 1
        # TODO option value: 9 (+ 3 pad bytes)
        # TODO this will add 8 bytes to the block length
        return b"".join(struct.pack(WSHEND, i) for i in [0x00000001, 20, 227, 0, 20])
        # return b"".join(struct.pack(WSHEND, i) for i in [0x00000001, 20, 227 << 16, 0, 20])

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

        frame = struct.pack('>I', canid)  # CAN ID in big endian format
        frame += bytes([len(data), 0, 0, 0])  # Frame payload length, padding/reserved
        frame += padded_data

        # data_h = struct.unpack('>I', padded_data[:4])[0]
        # data_l = struct.unpack('>I', padded_data[4:])[0]

        epb_prolog = b"".join(struct.pack(WSHEND, i) for i in [0x00000006,
                                                               48,
                                                               interface_id,
                                                               timestamp >> 32,
                                                               timestamp & 0xffffffff,
                                                               16,
                                                               16])
        epb_epilog = struct.pack(WSHEND, 48)

        return epb_prolog + frame + epb_epilog


def decode_frame(f: bytes, use_pcapng: bool):
    while len(f) > 0:
        frame = f[:19]
        f = f[19:]
        if frame[0] & 0x80 == 0:
            dlc = frame[1]
            remote = True if frame[0] & 1 else False
            if remote:
                dlc = 0
            filter_index = frame[2]
            timestamp = struct.unpack('>I', frame[3:7])[0]
            canid = struct.unpack('>I', frame[7:11])[0]
            data = frame[11:11 + dlc]
            extended = canid & (1 << 29)
            ida = (canid >> 18) & 0x7ff
            idb = canid & 0x3ffff
            arbitration_id = canid & 0x1fffffff if extended else ida

            if use_pcapng:
                sys.stdout.buffer.write(CANPCAPNG.get_epb(ida=ida,
                                                          idb=idb,
                                                          ide=1 if extended else 0,
                                                          rtr=1 if remote else 0,
                                                          data=data,
                                                          timestamp=timestamp))
                sys.stdout.buffer.flush()
            else:
                if extended:
                    print("CAN frame: E{:08x} data={} remote={}, timestamp={}".format(arbitration_id, data.hex(), remote, timestamp))
                else:
                    print("CAN frame: S{:03x} data={} remote={}, timestamp={}".format(arbitration_id, data.hex(), remote, timestamp))
        else:
            if use_pcapng:
                pass
            else:
                print("Overflow")
    

def canmon(use_pcapng):
    min_handler = MINTransportSerial(port=MIN_PORT, loglevel=DEBUG)
    min_log_handler = FileHandler('min.log')
    min_logger.addHandler(min_log_handler)

    # Tell the target that we are resetting to start a session
    min_handler.transport_reset()

    if use_pcapng:
        sys.stdout.buffer.write(CANPCAPNG.get_shb())
        sys.stdout.buffer.write(CANPCAPNG.get_idb())
        sys.stdout.buffer.flush()

    while True:
        # Wait for frames
        for frame in wait_for_frames(min_handler):
            # print(">>>>>>>>>>>>>>>> Got frame min_id={}, payload len={}".format(frame.min_id, len(frame.payload)))
            if frame.min_id == 1:
                decode_frame(frame.payload, use_pcapng)


if __name__ == "__main__":
    canmon(use_pcapng=True)

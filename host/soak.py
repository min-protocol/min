"""
Soak test for transport layer
"""
from logging import DEBUG, FileHandler
from random import randrange
from struct import unpack
from time import time

from min import MINTransportSerial, min_logger


MIN_PORT = '/dev/tty.usbmodem1421'


def bytes_to_int32(data: bytes, big_endian=True) -> int:
    if len(data) != 4:
        raise ValueError("int32 shoud be exactly 4 bytes")
    if big_endian:
        return unpack('>I', data)[0]
    else:
        return unpack('<I', data)[0]


def wait_for_frames(min_handler: MINTransportSerial, timeout=3.0):
    start = time()
    while True:
        frames = min_handler.poll()
        if frames:
            return frames
        if time() - start > timeout:
            raise TimeoutError


def soak_test():
    min_handler = MINTransportSerial(port=MIN_PORT, loglevel=DEBUG)
    min_handler.fake_errors = True

    min_log_handler = FileHandler('min.log')
    min_logger.addHandler(min_log_handler)

    # Tell the target that we are resetting to start a session
    min_handler.transport_reset()

    while True:
        # Send up to 10 frames in a batch
        batch_len = randrange(10) + 1
        min_ids = []
        payloads = []
        print("Sending a batch of length {}".format(batch_len))
        total_payload_size = 512
        for run in range(batch_len):
            min_id = randrange(0x40)
            payload_size = min(total_payload_size, randrange(256))
            total_payload_size -= payload_size
            payload = bytes([randrange(256) for i in range(payload_size)])
            min_ids.append(min_id)
            payloads.append(payload)
            # Send a frame on the serial line
            print(">>>>>>>>>>>>>>>> Sending ID={} payload len={}".format(min_id, payload_size))
            min_handler.queue_frame(min_id=min_id, payload=payload)

        while True:
            # Wait for the frames to come back
            for frame in wait_for_frames(min_handler):
                print(">>>>>>>>>>>>>>>> Got frame min_id={}, payload len={}".format(frame.min_id, len(frame.payload)))
                if frame.min_id != min_ids[0]:
                    raise AssertionError("Failed: did not get back the MIN ID we sent")
                if frame.payload != payloads[0]:
                    raise AssertionError("Failed: did not get back the payload we sent")
                del min_ids[0]
                del payloads[0]
                print("Remaining: {}".format(len(min_ids)))
            if len(min_ids) == 0:
                print("Batch received back OK")
                break


if __name__ == "__main__":
    soak_test()
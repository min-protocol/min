"""
Implementation of T-MIN for Python.
Designed to run on a host PC (the target board has a C version).

Author: Ken Tindell
Copyright (c) 2014-2017 JK Energy Ltd.
Licensed under MIT License.
"""
from random import SystemRandom
from struct import pack
from binascii import crc32
from threading import Lock
from time import time
from logging import getLogger, ERROR
from typing import Dict, Optional, List

from serial import Serial, SerialException


randomizer = SystemRandom()


min_logger = getLogger("min")


def int32_to_bytes(value: int) -> bytes:
    return pack(">I", value)


def bytes_to_hexstr(bytesequence: bytes) -> str:
    return "".join(f"{b:02X}" for b in bytesequence)


class MINConnectionError(Exception):
    pass


class MINFrame:
    def __init__(
        self, min_id: int, payload: bytes, seq: int, transport: bool, ack_or_reset=False
    ):
        if ack_or_reset:
            self.min_id = min_id
        else:
            self.min_id = min_id & 0x3F
        self.payload = payload
        self.seq = seq
        self.is_transport = transport
        # Time in ms
        self.last_sent_time: Optional[int] = None


class MINTransport:
    """
    Handle MIN Transport.

    Runs as a polled system; typically will be subclassed to run in a
    threaded environment that puts thread locks around API calls.
    """

    # Calls to bind this to a serial system in a host
    def _now_ms(self) -> int:
        raise NotImplementedError

    def _serial_write(self, data: bytes):
        raise NotImplementedError

    def _serial_any(self) -> bool:
        raise NotImplementedError

    def _serial_read_all(self) -> bytes:
        raise NotImplementedError

    def _serial_close(self):
        raise NotImplementedError

    ACK = 0xFF
    RESET = 0xFE

    HEADER_BYTE = 0xAA
    STUFF_BYTE = 0x55
    EOF_BYTE = 0x55

    SEARCHING_FOR_SOF = 0
    RECEIVING_ID_CONTROL = 1
    RECEIVING_LENGTH = 2
    RECEIVING_SEQ = 3
    RECEIVING_PAYLOAD = 4
    RECEIVING_CHECKSUM_3 = 5
    RECEIVING_CHECKSUM_2 = 6
    RECEIVING_CHECKSUM_1 = 7
    RECEIVING_CHECKSUM_0 = 8
    RECEIVING_EOF = 9

    def __init__(
        self,
        window_size=8,
        rx_window_size=16,
        transport_fifo_size=100,
        idle_timeout_ms=3000,
        ack_retransmit_timeout_ms=25,
        frame_retransmit_timeout_ms=50,
        loglevel=ERROR,
    ):
        """
        :param window_size: Number of outstanding unacknowledged frames
                            permitted to send
        :param rx_window_size: Number of outstanding unacknowledged frames
                               that can be received
        :param transport_fifo_size: Maximum number of outstanding frames to send
        :param idle_timeout_ms: Time before connection assumed to have been lost and
                                retransmissions stopped
        :param ack_retransmit_timeout_ms: Time before ACK frames are resent
        :param frame_retransmit_timeout_ms: Time before frames are resent
        :param loglevel: set the logging desired
        """
        self.transport_fifo_size = transport_fifo_size
        self.ack_retransmit_timeout_ms = ack_retransmit_timeout_ms
        self.max_window_size = window_size
        self.idle_timeout_ms = idle_timeout_ms
        self.frame_retransmit_timeout_ms = frame_retransmit_timeout_ms
        self.rx_window_size = rx_window_size

        min_logger.setLevel(level=loglevel)

        # Stats about the link
        self._longest_transport_fifo = 0
        self._dropped_frames = 0
        self._spurious_acks = 0
        self._mismatched_acks = 0
        self._duplicate_frames = 0
        self._retransmitted_frames = 0
        self._resets_received = 0
        self._sequence_mismatch_drops = 0

        # State of transport FIFO
        self._transport_fifo: List[MINFrame] = []
        self._last_sent_ack_time_ms: Optional[int] = None
        self._last_received_anything_ms: Optional[int] = None
        self._last_received_frame_ms: Optional[int] = None
        self._last_sent_frame_ms: Optional[int] = None

        # State for receiving a MIN frame
        self._rx_frame_buf = bytearray()
        self._rx_header_bytes_seen = 0
        self._rx_frame_state = self.SEARCHING_FOR_SOF
        self._rx_frame_checksum = 0
        self._rx_payload_bytes = bytearray()
        self._rx_frame_id_control = 0
        self._rx_frame_seq = 0
        self._rx_frame_length = 0
        self._rx_control = 0
        self._accepted_min_frames = []
        self._rx_list = []
        self._stashed_rx_dict: Dict[int, MINFrame] = {}

        # Sequence numbers
        self._rn = 0  # Sequence number expected to be received next
        self._sn_min = (
            0  # Sequence number of first frame currently in the sending window
        )
        self._sn_max = 0  # Next sequence number to use for sending a frame

        # NACK status
        self._nack_outstanding = None

        self._transport_fifo_reset()

    def _transport_fifo_pop(self):
        assert len(self._transport_fifo) > 0

        del self._transport_fifo[0]

    def _transport_fifo_get(self, n: int) -> MINFrame:
        return self._transport_fifo[n]

    def _transport_fifo_send(self, frame: MINFrame):
        on_wire_bytes = self._on_wire_bytes(frame=frame)
        frame.last_sent_time = self._now_ms()
        self._serial_write(on_wire_bytes)

    def _send_ack(self):
        # For a regular ACK we request no additional retransmits
        ack_frame = MINFrame(
            min_id=self.ACK,
            seq=self._rn,
            payload=bytes([self._rn]),
            transport=True,
            ack_or_reset=True,
        )
        on_wire_bytes = self._on_wire_bytes(frame=ack_frame)
        self._last_sent_ack_time_ms = self._now_ms()
        min_logger.debug("Sending ACK, seq=%d", ack_frame.seq)
        self._serial_write(on_wire_bytes)

    def _send_nack(self, to: int):
        # For a NACK we send an ACK but also request some frame retransmits
        nack_frame = MINFrame(
            min_id=self.ACK,
            seq=self._rn,
            payload=bytes([to]),
            transport=True,
            ack_or_reset=True,
        )
        on_wire_bytes = self._on_wire_bytes(frame=nack_frame)
        min_logger.debug("Sending NACK, seq=%d, to=%d", nack_frame.seq, to)
        self._serial_write(on_wire_bytes)

    def _send_reset(self):
        min_logger.debug("Sending RESET")
        reset_frame = MINFrame(
            min_id=self.RESET, seq=0, payload=bytes(), transport=True, ack_or_reset=True
        )
        on_wire_bytes = self._on_wire_bytes(frame=reset_frame)
        self._serial_write(on_wire_bytes)

    def _transport_fifo_reset(self):
        self._transport_fifo = []
        self._last_received_anything_ms = self._now_ms()
        self._last_sent_ack_time_ms = self._now_ms()
        self._last_sent_frame_ms = 0
        self._last_received_frame_ms = 0
        self._sn_min = 0
        self._sn_max = 0
        self._rn = 0

    def _rx_reset(self):
        self._stashed_rx_dict = {}
        self._rx_list = []

    def transport_reset(self):
        """
        Sends a RESET to the other side to say that we are going away
        and clears out the FIFO and receive queues
        :return:
        """
        self._send_reset()

        self._transport_fifo_reset()
        self._rx_reset()

    def send_frame(self, min_id: int, payload: bytes):
        """
        Sends a MIN frame with a given ID directly on the wire.

        Will be silently discarded if any line noise.
        :param min_id: ID of MIN frame (0 .. 63)
        :param payload: up to 255 bytes of payload
        :return:
        """
        if len(payload) not in range(256):
            raise ValueError("MIN payload too large")
        if min_id not in range(64):
            raise ValueError("MIN ID out of range")
        frame = MINFrame(min_id=min_id, payload=payload, transport=False, seq=0)
        on_wire_bytes = self._on_wire_bytes(frame=frame)
        min_logger.info(
            "Sending MIN frame, min_id=0x%02X, payload=%s",
            min_id,
            bytes_to_hexstr(payload),
        )
        min_logger.debug(
            "Sending MIN frame, on wire bytes=%s", bytes_to_hexstr(on_wire_bytes)
        )
        self._serial_write(on_wire_bytes)

    def queue_frame(self, min_id: int, payload: bytes):
        """
        Queues a MIN frame for transmission through the transport protocol.

        Will be retransmitted until it is
        delivered or the connection has timed out.

        :param min_id: ID of MIN frame (0 .. 63)
        :param payload: up to 255 bytes of payload
        :return:
        """
        if len(payload) not in range(256):
            raise ValueError("MIN payload too large")
        if min_id not in range(64):
            raise ValueError("MIN ID out of range")
        # Frame put into the transport FIFO
        if len(self._transport_fifo) < self.transport_fifo_size:
            min_logger.debug("Queueing min_id=0x%2X", min_id)
            frame = MINFrame(
                min_id=min_id, payload=payload, seq=self._sn_max, transport=True
            )
            self._transport_fifo.append(frame)
        else:
            self._dropped_frames += 1
            raise MINConnectionError("No space in transport FIFO queue")

    def _min_frame_received(
        self, min_id_control: int, min_payload: bytes, min_seq: int
    ):
        """
        Handle a received MIN frame.

        Because this runs on a host with plenty of CPU time and memory
        we stash out-of-order frames and send negative acknowledgements (NACKs) to ask
        sfor missing ones. This greatly improves the performance in the presence
        of line noise: a dropped frame will be specifically requested to be resent a
        nd then the stashed frames appended in the right order.

        Note that the automatic retransmit of frames must be tuned carefully so that
        a window + a NACK received + retransmission of missing frames + ACK for the
        complete set is faster than the retransmission timeout otherwise there is
        unnecessary retransmission of frames which wastes bandwidth.

        The embedded version of this code does not implement NACKs: generally the MCU
        will not have enough memory to stash out-of-order frames for later reassembly.
        """
        min_logger.debug(
            "MIN frame received @%f: min_id_control=0x%02X, min_seq=%d",
            time(),
            min_id_control,
            min_seq,
        )
        self._last_received_anything_ms = self._now_ms()
        if min_id_control & 0x80:
            if min_id_control == self.ACK:
                min_logger.debug("Received ACK")
                # The ACK number indicates the serial number of the next packet wanted,
                # so any previous packets can be marked off
                number_acked = (min_seq - self._sn_min) & 0xFF
                number_in_window = (self._sn_max - self._sn_min) & 0xFF
                # Need to guard against old ACKs from an old session still turning up.
                # Number acked will be 1 if there are no frames in the window
                if number_acked <= number_in_window:
                    min_logger.debug("Number ACKed = %d", number_acked)
                    self._sn_min = min_seq

                    assert len(self._transport_fifo) >= number_in_window
                    assert number_in_window <= self.max_window_size

                    new_number_in_window = (self._sn_max - self._sn_min) & 0xFF
                    if new_number_in_window + number_acked != number_in_window:
                        raise AssertionError

                    for i in range(number_acked):
                        self._transport_fifo_pop()
                else:
                    if number_in_window > 0:
                        msg = (
                            "Spurious ACK: self._sn_min=%d, self._sn_max=%d, "
                            "min_seq=%d, payload[0]=0x%02X"
                        )
                        min_logger.warning(
                            msg,
                            self._sn_min,
                            self._sn_max,
                            min_seq,
                            min_payload[0],
                        )
                    self._spurious_acks += 1
            elif min_id_control == self.RESET:
                min_logger.debug("RESET received %d", min_seq)
                self._resets_received += 1
                self._transport_fifo_reset()
                self._rx_reset()
            else:
                # MIN frame received
                min_frame = MINFrame(
                    min_id=min_id_control,
                    payload=min_payload,
                    seq=min_seq,
                    transport=True,
                )

                self._last_received_frame_ms = self._now_ms()
                if min_seq == self._rn:
                    min_logger.debug(
                        "MIN application frame received @%f (min_id=0x%02X seq=%d)",
                        time(),
                        min_id_control & 0x3F,
                        min_seq,
                    )
                    self._rx_list.append(min_frame)

                    # We want this frame. Now see if there are stashed frames
                    # it joins up with and 'receive' those
                    self._rn = (self._rn + 1) & 0xFF
                    while self._rn in self._stashed_rx_dict:
                        stashed_frame = self._stashed_rx_dict[self._rn]
                        msg = (
                            "MIN application stashed frame recovered @%f "
                            "(self._rn=%d min_id=0x%02X seq=%d)"
                        )
                        min_logger.debug(
                            msg,
                            time(),
                            self._rn,
                            stashed_frame.min_id,
                            stashed_frame.seq,
                        )
                        del self._stashed_rx_dict[self._rn]
                        self._rx_list.append(stashed_frame)
                        self._rn = (self._rn + 1) & 0xFF
                        if self._rn == self._nack_outstanding:
                            self._nack_outstanding = None  # The missing frames we asked
                            # for have joined up with the main sequence

                    # If there are stashed frames left then it means that
                    # the stashed ones have missing frames in the sequence
                    if (
                        self._nack_outstanding is None
                        and len(self._stashed_rx_dict) > 0
                    ):
                        # We can send a NACK to ask for those too, starting with
                        # the earliest sequence number
                        earliest_seq = sorted(self._stashed_rx_dict.keys())[0]
                        # Check it's within the window size from us
                        if (earliest_seq - self._rn) & 0xFF < self.rx_window_size:
                            self._nack_outstanding = earliest_seq
                            self._send_nack(earliest_seq)
                        else:
                            # Something has gone wrong here: stale stuff is
                            # hanging around, give up and reset
                            min_logger.error(
                                "Stale frames in the stashed area; resetting"
                            )
                            self._nack_outstanding = None
                            self._stashed_rx_dict = {}
                            self._send_ack()
                    else:
                        self._send_ack()
                    min_logger.debug(
                        "Sending ACK for min ID=0x%02X with self._rn=%d",
                        min_id_control & 0x3F,
                        self._rn,
                    )
                else:
                    # If the frames come within the window size in the future sequence
                    # range then we accept them and assume some were missing
                    # (They may also be duplicates, in which case we store them over
                    # sthe top of the old ones)
                    if (min_seq - self._rn) & 0xFF < self.rx_window_size:
                        # We want to only NACK a range of frames once,
                        # not each time otherwise we will overload with retransmissions
                        if self._nack_outstanding is None:
                            # If we are missing specific frames then send a NACK to
                            # specifically request them
                            min_logger.debug(
                                "Sending NACK for min ID=0x%02X with seq=%d to=%d",
                                min_id_control & 0x3F,
                                self._rn,
                                min_seq,
                            )
                            self._send_nack(min_seq)
                            self._nack_outstanding = min_seq
                        else:
                            min_logger.debug("(Outstanding NACK)")

                        # Hang on to this frame because we will join it up later with
                        # the missing ones that are re-sent
                        self._stashed_rx_dict[min_seq] = min_frame
                        msg = (
                            "MIN application frame stashed @%f (min_id=0x%02X, seq=%d)"
                        )
                        min_logger.debug(msg, time(), min_id_control & 0x3F, min_seq)
                    else:
                        min_logger.warning(
                            "Frame stale? Discarding @%f (min_id=0x%02X, seq=%d)",
                            time(),
                            min_id_control & 0x3F,
                            min_seq,
                        )
                        if (
                            min_seq in self._stashed_rx_dict
                            and min_payload != self._stashed_rx_dict[min_seq].payload
                        ):
                            min_logger.error("Inconsistency between frame contents")
                        # Out of range
                        # (may be an old retransmit duplicate that we don't want)
                        # - throw it away
                        self._sequence_mismatch_drops += 1
        else:
            min_frame = MINFrame(
                min_id=min_id_control, payload=min_payload, seq=0, transport=False
            )
            self._rx_list.append(min_frame)

    def _rx_bytes(self, data: bytes):
        """
        Called by handler to pass over a sequence of bytes
        :param data:
        """
        min_logger.debug("Received bytes: %s", bytes_to_hexstr(data))
        for byte in data:
            if self._rx_header_bytes_seen == 2:
                self._rx_header_bytes_seen = 0
                if byte == self.HEADER_BYTE:
                    self._rx_frame_state = self.RECEIVING_ID_CONTROL
                    continue
                if byte == self.STUFF_BYTE:
                    # Discard this byte; carry on receiving the next character
                    continue
                # By here something must have gone wrong, give up on this frame and
                # look for new header
                self._rx_frame_state = self.SEARCHING_FOR_SOF
                continue

            if byte == self.HEADER_BYTE:
                self._rx_header_bytes_seen += 1
            else:
                self._rx_header_bytes_seen = 0

            if self._rx_frame_state == self.SEARCHING_FOR_SOF:
                pass
            elif self._rx_frame_state == self.RECEIVING_ID_CONTROL:
                self._rx_frame_id_control = byte
                self._rx_payload_bytes = 0
                if self._rx_frame_id_control & 0x80:
                    self._rx_frame_state = self.RECEIVING_SEQ
                else:
                    self._rx_frame_state = self.RECEIVING_LENGTH
            elif self._rx_frame_state == self.RECEIVING_SEQ:
                self._rx_frame_seq = byte
                self._rx_frame_state = self.RECEIVING_LENGTH
            elif self._rx_frame_state == self.RECEIVING_LENGTH:
                self._rx_frame_length = byte
                self._rx_control = byte
                self._rx_frame_buf = bytearray()
                if self._rx_frame_length > 0:
                    self._rx_frame_state = self.RECEIVING_PAYLOAD
                else:
                    self._rx_frame_state = self.RECEIVING_CHECKSUM_3
            elif self._rx_frame_state == self.RECEIVING_PAYLOAD:
                self._rx_frame_buf.append(byte)
                self._rx_frame_length -= 1
                if self._rx_frame_length == 0:
                    self._rx_frame_state = self.RECEIVING_CHECKSUM_3
            elif self._rx_frame_state == self.RECEIVING_CHECKSUM_3:
                self._rx_frame_checksum = byte << 24
                self._rx_frame_state = self.RECEIVING_CHECKSUM_2
            elif self._rx_frame_state == self.RECEIVING_CHECKSUM_2:
                self._rx_frame_checksum |= byte << 16
                self._rx_frame_state = self.RECEIVING_CHECKSUM_1
            elif self._rx_frame_state == self.RECEIVING_CHECKSUM_1:
                self._rx_frame_checksum |= byte << 8
                self._rx_frame_state = self.RECEIVING_CHECKSUM_0
            elif self._rx_frame_state == self.RECEIVING_CHECKSUM_0:
                self._rx_frame_checksum |= byte
                if self._rx_frame_id_control & 0x80:
                    computed_checksum = self._crc32(
                        bytearray(
                            [
                                self._rx_frame_id_control,
                                self._rx_frame_seq,
                                self._rx_control,
                            ]
                        )
                        + self._rx_frame_buf
                    )
                else:
                    computed_checksum = self._crc32(
                        bytearray([self._rx_frame_id_control, self._rx_control])
                        + self._rx_frame_buf
                    )

                if self._rx_frame_checksum != computed_checksum:
                    min_logger.warning(
                        "CRC mismatch (0x%08X vs 0x%08X), frame dropped",
                        self._rx_frame_checksum,
                        computed_checksum,
                    )
                    # Frame fails checksum, is dropped
                    self._rx_frame_state = self.SEARCHING_FOR_SOF
                else:
                    # Checksum passes, wait for EOF
                    self._rx_frame_state = self.RECEIVING_EOF
            elif self._rx_frame_state == self.RECEIVING_EOF:
                if byte == self.EOF_BYTE:
                    # Frame received OK, pass up frame for handling")
                    self._min_frame_received(
                        min_id_control=self._rx_frame_id_control,
                        min_payload=bytes(self._rx_frame_buf),
                        min_seq=self._rx_frame_seq,
                    )
                else:
                    min_logger.warning("No EOF received, dropping frame")

                # Look for next frame
                self._rx_frame_state = self.SEARCHING_FOR_SOF
            else:
                min_logger.error("Unexpected state, state machine reset")
                # Should never get here but in case we do just reset
                self._rx_frame_state = self.SEARCHING_FOR_SOF

    def _on_wire_bytes(self, frame: MINFrame) -> bytes:
        """
        Get the on-wire byte sequence for the frame,

        sincluding stuff bytes after every 0xaa 0xaa pair
        """
        if frame.is_transport:
            prolog = (
                bytes([frame.min_id | 0x80, frame.seq, len(frame.payload)])
                + frame.payload
            )
        else:
            prolog = bytes([frame.min_id, len(frame.payload)]) + frame.payload

        crc = crc32(prolog, 0)
        raw = prolog + int32_to_bytes(crc)

        stuffed = bytearray([self.HEADER_BYTE, self.HEADER_BYTE, self.HEADER_BYTE])

        count = 0

        for i in raw:
            stuffed.append(i)
            if i == self.HEADER_BYTE:
                count += 1
                if count == 2:
                    stuffed.append(self.STUFF_BYTE)
                    count = 0
            else:
                count = 0

        stuffed.append(self.EOF_BYTE)

        return bytes(stuffed)

    @staticmethod
    def _crc32(checksummed_data: bytearray, start=0xFFFFFFFF):
        """
        The 'manual' implementation is left here as a guide to implementing this on
        microcontrollers. It's cross-checked with the standard Python library version.
        """
        crc = start
        for byte in checksummed_data:
            crc ^= byte
            for j in range(8):
                mask = -(crc & 1)
                crc = (crc >> 1) ^ (0xEDB88320 & mask)
        checksum = ~crc % (1 << 32)

        if checksum != crc32(checksummed_data, 0):
            raise AssertionError("CRC algorithm mismatch")

        return checksum

    def transport_stats(self):
        """
        Returns a tuple of all the transport stats
        """
        return (
            self._longest_transport_fifo,
            self._last_sent_frame_ms,
            self._sequence_mismatch_drops,
            self._retransmitted_frames,
            self._resets_received,
            self._duplicate_frames,
            self._mismatched_acks,
            self._spurious_acks,
        )

    def _find_oldest_frame(self):
        if len(self._transport_fifo) == 0:
            raise AssertionError

        window_size = (self._sn_max - self._sn_min) & 0xFF
        oldest_frame = self._transport_fifo[0]  # type: MINFrame
        longest_elapsed_time = self._now_ms() - oldest_frame.last_sent_time

        for i in range(window_size):
            elapsed = self._now_ms() - self._transport_fifo[i].last_sent_time
            if elapsed >= longest_elapsed_time:
                oldest_frame = self._transport_fifo[i]
                longest_elapsed_time = elapsed

        return oldest_frame

    def poll(self):
        """
        Polls the serial line.

        Runs through MIN, sends ACKs, handles retransmits where ACK has gone missing.

        :return: array of accepted MIN frames
        """
        remote_connected = (
            self._now_ms() - self._last_received_anything_ms
        ) < self.idle_timeout_ms
        remote_active = (
            self._now_ms() - self._last_received_frame_ms
        ) < self.idle_timeout_ms

        self._rx_list = []

        data = self._serial_read_all()
        if data:
            self._rx_bytes(data=data)

        window_size = (self._sn_max - self._sn_min) & 0xFF
        if (
            window_size < self.max_window_size
            and len(self._transport_fifo) > window_size
        ):
            # Frames still to send
            frame = self._transport_fifo_get(n=window_size)
            frame.seq = self._sn_max
            self._last_sent_frame_ms = self._now_ms()
            frame.last_sent_time = self._now_ms()
            min_logger.debug(
                "Sending new frame id=0x%02X seq=%d len=%d payload=%s",
                frame.min_id,
                frame.seq,
                len(frame.payload),
                bytes_to_hexstr(frame.payload),
            )
            self._transport_fifo_send(frame=frame)
            self._sn_max = (self._sn_max + 1) & 0xFF
        else:
            # Maybe retransmits
            if window_size > 0 and remote_connected:
                oldest_frame = self._find_oldest_frame()
                if (
                    self._now_ms() - oldest_frame.last_sent_time
                    > self.frame_retransmit_timeout_ms
                ):
                    min_logger.debug(
                        "Resending old frame id=0x%02X seq=%d",
                        oldest_frame.min_id,
                        oldest_frame.seq,
                    )
                    self._transport_fifo_send(frame=oldest_frame)

        # Periodically transmit ACK
        if (
            self._now_ms() - self._last_sent_ack_time_ms
            > self.ack_retransmit_timeout_ms
        ):
            if remote_active:
                min_logger.debug("Periodic send of ACK")
                self._send_ack()

        if (self._sn_max - self._sn_max) & 0xFF > window_size:
            raise AssertionError

        return self._rx_list

    def close(self):
        self._serial_close()


class MINTransportSerial(MINTransport):
    """
    Bound to Pyserial driver.

    But not thread safe: must not call poll() and send() at the same time.
    """

    def _corrupted_data(self, data: bytes):
        """
        Randomly perturb a bit in one in 1000 bytes
        :param data:
        :return:
        """
        corrupted_data = []
        for byte in data:
            r = randomizer.random()
            if r < 0.00005:
                min_logger.debug("r=%f", r)
                new_byte = byte ^ (1 << randomizer.randrange(8))
                min_logger.info("Corrupted (=0x%02X}, was=0x%02X)", byte, new_byte)
                corrupted_data.append(new_byte)
            else:
                corrupted_data.append(byte)
        return bytes(corrupted_data)

    def _now_ms(self):
        now = int(time() * 1000.0)
        return now

    def _serial_write(self, data: bytes):
        if self.fake_errors:
            data = self._corrupted_data(data)

        min_logger.debug("_serial_write: %s", bytes_to_hexstr(data))
        self._serial.write(data)

    def _serial_any(self):
        return self._serial.in_waiting > 0

    def _serial_read_all(self):
        data = self._serial.read_all()
        if self.fake_errors:
            data = self._corrupted_data(data)
        return data

    def _serial_close(self):
        self._serial.close()

    def __init__(self, port, baudrate=9600, loglevel=ERROR):
        """
        Open MIN connection on a given port.
        :param port: serial port
        :param debug:
        """
        self.fake_errors = False
        try:
            self._serial = Serial(port=port, baudrate=baudrate, timeout=0.1, write_timeout=1.0)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except SerialException:
            raise MINConnectionError(f"Transport MIN cannot open port '{port}'")
        super().__init__(loglevel=loglevel)


class ThreadsafeTransportMINSerialHandler(MINTransportSerial):
    """
    This class wraps the API calls with thread locks

    to prevent concurrent access to the system.
    A typical usage is to create a simple thread that calls poll() in a loop
    which takes MIN frames received and puts them into a Python queue.
    The application can send directly and pick up incoming frames from the queue.
    """

    def __init__(self, port, loglevel=ERROR):
        super().__init__(port=port, loglevel=loglevel)
        self._thread_lock = Lock()

    def close(self):
        self._thread_lock.acquire()
        try:
            super().close()
        except Exception as e:
            self._thread_lock.release()
            raise e
        self._thread_lock.release()

    def transport_stats(self):
        self._thread_lock.acquire()
        try:
            result = super().transport_stats()
        except Exception as e:
            self._thread_lock.release()
            raise e
        self._thread_lock.release()

        return result

    def send_frame(self, min_id: int, payload: bytes):
        self._thread_lock.acquire()
        try:
            super().send_frame(min_id=min_id, payload=payload)
        except Exception as e:
            self._thread_lock.release()
            raise e
        self._thread_lock.release()

    def queue_frame(self, min_id: int, payload: bytes):
        self._thread_lock.acquire()
        try:
            super().queue_frame(min_id=min_id, payload=payload)
        except Exception as e:
            self._thread_lock.release()
            raise e
        self._thread_lock.release()

    def poll(self):
        self._thread_lock.acquire()
        try:
            result = super().poll()
        except Exception as e:
            self._thread_lock.release()
            raise e
        self._thread_lock.release()

        return result

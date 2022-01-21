"""
A MIN-CAN monitor for the CANPico
"""
from rp2 import MIN
import machine
import binascii


class CANMonitor(MINMonitor):
    """
    Class that handles CAN frames over MIN
    """
    INIT_CAN = 4  # Set up the CAN controller
    SET_TRIGGER = 5  # Set the TRIG pin trigger conditions
    SEND_CAN = 6  # Send a CAN frame
    SEND_CAN_FIFO = 7  # Send a CAN frame

    # Outgoing MIN frames
    CAN_RECV = 11  # Carry details of received frames
    CAN_TX_RECV = 12  # Carry details of transmitted frames

    def __init__(self):
        self.super()
        self.c = None  # CAN controller
        self.handlers[self.INIT_CAN] = self.init_can
        self.handlers[self.SET_TRIGGER] = self.set_trigger
        self.handlers[self.SEND_CAN] = self.send_can
        self.handlers[self.SEND_CAN_FIFO] = self.send_can_fifo

    def loop(self):
        super().loop()
        if self.c is not None:
            # Send up the line outstanding CAN frames/errors
            can_recv_bytes = self.c.recv(as_bytes=True)
            if can_recv_bytes:
                self.min.send_frame(self.CAN_RECV, can_recv_bytes)
            # Send up the line outstanding CAN transmit events
            can_tx_recv_bytes = self.c.recv_tx_events(as_bytes=True)
            if can_tx_recv_bytes:
                self.min_send_frame(self.CAN_TX_RECV, can_tx_recv_bytes)

    def send_can(self, min_payload):
        if self.c is not None:
            frames = CANFrame.from_bytes(min_payload)
            c.send_frames(frames)

    def send_can_fifo(self, min_payload):
        if self.c is not None:
            frames = CANFrame.from_bytes(min_payload)
            c.send_frames(frames, fifo=True)

    def init_can(self, min_payload):
        self.c = CAN()  # TODO placeholder: replace with full CAN parameter settings
        return None

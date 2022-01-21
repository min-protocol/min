from time import sleep
from min import ThreadsafeTransportMINSerialHandler
from threading import Thread, Event
from queue import Queue, Empty


class MINMonitor:
    """
    Base class for communicating with a remote board over MIN

    Uses a thread to drive the MIN state machine, and queues to send MIN frames
    and receive them back. There can be more than one receive queue: the MIN ID can be used
    to put certain messages into their own queues.
    """
    PING = 1  # Send back the frame
    IDENTIFY = 2  # Identify this hardware
    STOP = 3  # Stop running the monitor
    PRINT = 4  # Debug print to the REPL console
    EXCEPTION = 10  # A command failed with an exception; report the exception to the host

    def __init__(self, port='/dev/ttyACM1'):
        self._min_handler = ThreadsafeTransportMINSerialHandler(port=port)
        self._thread = None  # Type: Thread
        self._stop_event = Event()
        self._is_stopped = False
        self._recv_messages = Queue()  # Used to pick up messages
        self._filter_messages = {}  # Messages can be redirected into specific queues
        self._send_messages = Queue()  # Used to queue outgoing messages
        self._start()

    def _thread_fn(self):
        """
        This is the main MIN thread, drives the MIN state machine, sends frames to the remote
        monitor and stores the frames coming back
        """
        while not self._stop_event.is_set():
            if self._min_handler is not None:
                # Drive state machine and receive MIN messages
                try:
                    msgs = self._min_handler.poll()
                    for msg in msgs:
                        if msg.min_id in self._filter_messages:
                            self._filter_messages[msg.min_id].put(msg)
                        else:
                            self._recv_messages.put(msg)
                    sleep(0.0)
                except Exception as e:
                    print("Exception: {}".format(e))
                    self._is_stopped = True
                    self._min_handler = None
                    return
                # Send an outgoing MIN message
                try:
                    min_id, min_payload = self._send_messages.get(block=False)
                    self._min_handler.queue_frame(min_id, min_payload)
                except Empty:
                    pass

    def _start(self):
        self._stop_event.clear()
        self._min_handler.transport_reset()
        self._thread = Thread(target=self._thread_fn)
        self._thread.start()

    def stop(self):
        if not self._is_stopped:
            self._stop_event.set()
            self._thread.join()
            if self._min_handler is not None:
                self._min_handler.close()
                self._min_handler = None
            self._is_stopped = True

    def recv(self, min_id=None, block=True, timeout=0.1):
        """
        Returns a received MIN message, optionally blocking until one turns up
        """
        msg = None
        try:
            if min_id is not None:
                msg = self._filter_messages[min_id].get(block=block, timeout=timeout)
            else:
                msg = self._recv_messages.get(block=block, timeout=timeout)
        except Empty:
            pass

        return msg

    def create_recv_filter(self, min_id):
        self._filter_messages[min_id] = Queue()

    def send_frame(self, min_id, min_payload):
        self._send_messages.put(item=(min_id, min_payload))

    def identify(self):
        if self.IDENTIFY not in self._filter_messages.keys():
            self.create_recv_filter(self.IDENTIFY)
        self.send_frame(self.IDENTIFY, b'hello')
        reply = self.recv(min_id=self.IDENTIFY)  # Blocks with timeout
        if reply is None:
            return None
        else:
            return reply.payload

    def ping(self, message=b'hello'):
        if self.PING not in self._filter_messages.keys():
            self.create_recv_filter(self.PING)
        self.send_frame(self.PING, message)
        reply = self.recv(self.PING)  # Blocks with timeout
        if reply is None:
            return None
        else:
            return reply.payload

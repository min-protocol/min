__author__ = 'ken'

import re


class MINObject:
    handles = {}

    c_ident = re.compile(r"^[^\d\W]\w*\Z", re.UNICODE)

    @classmethod
    def exists(cls, handle):
        if handle in cls.handles:
            return True
        else:
            return False

    @classmethod
    def get(cls, handle):
        if cls.exists(handle):
            return cls.handles[handle]
        else:
            raise ValueError("Handle '{}' unknown".format(handle))

    def __init__(self, handle):
        if MINObject.exists(handle):
            raise ValueError("Object with handle '{}' already exists".format(handle))
        if type(handle) == str and re.match(self.c_ident, handle):
            self.handles[handle] = self
        else:
            raise ValueError("Handle '{}' is not a valid C identifier".format(handle))
        self.handle = handle


class Signal(MINObject):
    signals = []
    c_types = {
        "uint8_t": {"bytes": 1, "signed": False},
        "uint16_t": {"bytes": 2, "signed": False},
        "uint32_t": {"bytes": 4, "signed": False},
        "int8_t": {"bytes": 1, "signed": True},
        "int16_t": {"bytes": 2, "signed": True},
        "int32_t": {"bytes": 4, "signed": True},
    }

    def __init__(self, json):
        c_type = json.get("c_type")
        handle = json.get("handle")
        update_bit = json.get("update_bit")
        display_name = json.get("display_name")
        definition = json.get("definition")

        super().__init__(handle=handle)

        if type(c_type) == str and c_type in self.c_types:
            self.c_type = c_type
        else:
            raise ValueError("Invalid c_type")
        self.c_type = c_type
        self.update_bit = update_bit or False
        self.display_name = type(display_name) == str and display_name or self.handle
        self.definition = type(definition) == str and definition or ""

        self.frame = None
        self.update_bit_number = 0
        self.byte_offset = None
        self.size = self.c_types[c_type]["bytes"]
        self.signed = self.c_types[c_type]["signed"]

        self.signals.append(self)

    def update_bit_mask(self):
        return 1 << (self.update_bit_number % 8)

    def update_bit_byte(self):
        return int(self.update_bit_number / 8)

    def is_read(self):
        return self.frame.input

    def is_written(self):
        return self.frame.output

    def variable_name(self):
        return "min_" + self.handle + "_var"

    def update_byte_name(self):
        return self.frame.update_byte_name_prefix() + str(self.update_bit_byte())

    def signed_char(self):
        if self.signed:
            return ""
        else:
            return "U"


class Frame(MINObject):
    min_ids = {}
    frames = []
    min_period = None

    def allocate_min_id(self):
        for min_id in range(0, 254):
            if min_id not in self.min_ids:
                self.min_id = min_id
                self.min_ids[min_id] = self
                return
        raise ValueError("No free MIN IDs")

    def get_period(self):
        if self.output:
            if self.signal_frame() or self.period_guarded():
                return int(self.period_ms / self.min_period) + 1
        else:
            raise ValueError("Frame period only valid for output frames")

    def get_offset(self):
        if self.output:
            if self.signal_frame():
                return int(self.offset_ms / self.min_period)
            else:
                raise ValueError("Offset not valid for raw or force-transmit frames")
        else:
            raise ValueError("Frame offset only valid for output frames")

    @staticmethod
    def allocate_min_ids():
        for f in Frame.frames:
            if f.min_id is None:
                f.allocate_min_id()

    def period_counter_name(self):
        return "min_" + self.handle + "_period_counter"

    def period_guarded(self):
        if (self.raw or self.force_transmit_signal) and self.period_ms is not None:
            return True
        else:
            return False

    def signal_frame(self):
        if self.force_transmit_signal or self.raw:
            return False
        else:
            return True

    def __init__(self, json):
        handle = json.get("handle")
        min_id = json.get("min_id")
        signal_handles = json.get("signals")
        display_name = json.get("display_name")
        period_ms = json.get("period_ms")
        offset_ms = json.get("offset_ms")
        raw = json.get("raw")
        force_transmit = json.get("force_transmit")

        super().__init__(handle=handle)

        if type(min_id) == int:
            if min_id == 255:
                raise ValueError("MIN ID of 255 is reserved in MIN 1.0")
            if min_id < 0 or min_id > 254:
                raise ValueError("MIN ID must be in range 0 .. 254 (255 is reserved)")
            self.min_id = min_id
            self.min_ids[min_id] = self
        else:
            self.min_id = None

        self.force_transmit_signal = force_transmit and Signal.get(force_transmit) or None
        self.signals = self.force_transmit_signal and [self.force_transmit_signal] or []
        for signal_handle in signal_handles or []:
            signal = Signal.get(signal_handle)
            if signal.frame:
                raise ValueError("Signal {} already allocated to frame {}".format(signal.handle, signal.frame.handle))
            else:
                signal.frame = self
                self.signals.append(signal)

        self.raw = raw and True
        if self.raw and self.signals != []:
            raise ValueError("Can't have signals ({}) in raw frame '{}'".format(self.signals, self.handle))

        self.display_name = type(display_name) == str and display_name or self.handle

        if type(period_ms) is int:
            period_ms = float(period_ms)

        self.period_ms = type(period_ms) is float and period_ms or None

        if self.raw or self.force_transmit_signal:
            pass  # If period set then period protection wanted
        else:
            if self.period_ms is None:
                raise ValueError("Period for frame {} must be a non-zero".format(self.handle))
        self.offset_ms = type(offset_ms) == float and offset_ms or 0.0

        self.input = False
        self.output = False
        self.update_bit_bytes = 0
        self.frame_size = 0
        self.frames.append(self)

    def pack_signals(self):
        signal_bytes = 0
        update_bit_number = 0
        for signal in self.signals:
            if signal.update_bit:
                signal.update_bit_number = update_bit_number
                update_bit_number += 1
                signal.byte_offset = signal_bytes
                signal_bytes += signal.size
        self.update_bit_bytes = int(1 + (update_bit_number - 1) / 8)
        self.frame_size = self.update_bit_bytes + signal_bytes

    def update_byte_name_prefix(self):
        return "min_" + self.handle + "_update_byte_"

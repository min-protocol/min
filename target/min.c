// Copyright (c) 2014-2017 JK Energy Ltd.
//
// Use authorized under the MIT license.

#include "min.h"

#define TRANSPORT_FIFO_SIZE_FRAMES_MASK             ((uint8_t)((1U << TRANSPORT_FIFO_SIZE_FRAMES_BITS) - 1U))
#define TRANSPORT_FIFO_SIZE_FRAME_DATA_MASK         ((uint16_t)((1U << TRANSPORT_FIFO_SIZE_FRAME_DATA_BITS) - 1U))

// Number of bytes needed for a frame with a given payload length, excluding stuff bytes
// 3 header bytes, ID/control byte, length byte, seq byte, 4 byte CRC, EOF byte
#define ON_WIRE_SIZE(p)                             ((p) + 11U)

// Special protocol bytes
enum {
    HEADER_BYTE = 0xaaU,
    STUFF_BYTE = 0x55U,
    EOF_BYTE = 0x55U,
};

// Receiving state machine
enum {
    SEARCHING_FOR_SOF,
    RECEIVING_ID_CONTROL,
    RECEIVING_SEQ,
    RECEIVING_LENGTH,
    RECEIVING_PAYLOAD,
    RECEIVING_CHECKSUM_3,
    RECEIVING_CHECKSUM_2,
    RECEIVING_CHECKSUM_1,
    RECEIVING_CHECKSUM_0,
    RECEIVING_EOF,
};

#ifdef TRANSPORT_PROTOCOL

#ifndef TRANSPORT_ACK_RETRANSMIT_TIMEOUT_MS
#define TRANSPORT_ACK_RETRANSMIT_TIMEOUT_MS         (25U)
#endif
#ifndef TRANSPORT_FRAME_RETRANSMIT_TIMEOUT_MS
#define TRANSPORT_FRAME_RETRANSMIT_TIMEOUT_MS       (50U) // Should be long enough for a whole window to be transmitted plus an ACK / NACK to get back
#endif
#ifndef TRANSPORT_MAX_WINDOW_SIZE
#define TRANSPORT_MAX_WINDOW_SIZE                   (16U)
#endif
#ifndef TRANSPORT_IDLE_TIMEOUT_MS
#define TRANSPORT_IDLE_TIMEOUT_MS                   (1000U)
#endif

enum {
    // Top bit must be set: these are for the transport protocol to use
    // 0x7f and 0x7e are reserved MIN identifiers.
    ACK = 0xffU,
    RESET = 0xfeU,
};

// Where the payload data of the frame FIFO is stored
uint8_t payloads_ring_buffer[TRANSPORT_FIFO_MAX_FRAME_DATA];

static uint32_t now;
static void send_reset(struct min_context *self);
#endif

static void crc32_init_context(struct crc32_context *context)
{
    context->crc = 0xffffffffU;
}

static void crc32_step(struct crc32_context *context, uint8_t byte)
{
    context->crc ^= byte;
    for(uint32_t j = 0; j < 8; j++) {
        uint32_t mask = (uint32_t) -(context->crc & 1U);
        context->crc = (context->crc >> 1) ^ (0xedb88320U & mask);
    }
}

static uint32_t crc32_finalize(struct crc32_context *context)
{
    return ~context->crc;
}


static void stuffed_tx_byte(struct min_context *self, uint8_t byte)
{
    // Transmit the byte
    min_tx_byte(self->port, byte);
    crc32_step(&self->tx_checksum, byte);

    // See if an additional stuff byte is needed
    if(byte == HEADER_BYTE) {
        if(--self->tx_header_byte_countdown == 0) {
            min_tx_byte(self->port, STUFF_BYTE);        // Stuff byte
            self->tx_header_byte_countdown = 2U;
        }
    }
    else {
        self->tx_header_byte_countdown = 2U;
    }
}

static void on_wire_bytes(struct min_context *self, uint8_t id_control, uint8_t seq, uint8_t *payload_base, uint16_t payload_offset, uint16_t payload_mask, uint8_t payload_len)
{
    uint8_t n, i;
    uint32_t checksum;

    self->tx_header_byte_countdown = 2U;
    crc32_init_context(&self->tx_checksum);

    min_tx_start(self->port);

    // Header is 3 bytes; because unstuffed will reset receiver immediately
    min_tx_byte(self->port, HEADER_BYTE);
    min_tx_byte(self->port, HEADER_BYTE);
    min_tx_byte(self->port, HEADER_BYTE);

    stuffed_tx_byte(self, id_control);
    if(id_control & 0x80U) {
        // Send the sequence number if it is a transport frame
        stuffed_tx_byte(self, seq);
    }

    stuffed_tx_byte(self, payload_len);

    for(i = 0, n = payload_len; n > 0; n--, i++) {
        stuffed_tx_byte(self, payload_base[payload_offset]);
        payload_offset++;
        payload_offset &= payload_mask;
    }

    checksum = crc32_finalize(&self->tx_checksum);

    // Network order is big-endian. A decent C compiler will spot that this
    // is extracting bytes and will use efficient instructions.
    stuffed_tx_byte(self, (uint8_t)((checksum >> 24) & 0xffU));
    stuffed_tx_byte(self, (uint8_t)((checksum >> 16) & 0xffU));
    stuffed_tx_byte(self, (uint8_t)((checksum >> 8) & 0xffU));
    stuffed_tx_byte(self, (uint8_t)((checksum >> 0) & 0xffU));

    // Ensure end-of-frame doesn't contain 0xaa and confuse search for start-of-frame
    min_tx_byte(self->port, EOF_BYTE);

    min_tx_finished(self->port);
}

#ifdef TRANSPORT_PROTOCOL

// Pops frame from front of queue, reclaims its ring buffer space
static void transport_fifo_pop(struct min_context *self)
{
#ifdef ASSERTION_CHECKING
    assert(self->transport_fifo.n_frames != 0);
#endif
    struct transport_frame *frame = &self->transport_fifo.frames[self->transport_fifo.head_idx];
    min_debug_print("Popping frame id=%d seq=%d\n", frame->min_id, frame->seq);

#ifdef ASSERTION_CHECKING
    assert(self->transport_fifo.n_ring_buffer_bytes >= frame->payload_len);
#endif

    self->transport_fifo.n_frames--;
    self->transport_fifo.head_idx++;
    self->transport_fifo.head_idx &= TRANSPORT_FIFO_SIZE_FRAMES_MASK;
    self->transport_fifo.n_ring_buffer_bytes -= frame->payload_len;
}

// Claim a buffer slot from the FIFO. Returns 0 if there is no space.
static struct transport_frame *transport_fifo_push(struct min_context *self, uint16_t data_size)
{
    // A frame is only queued if there aren't too many frames in the FIFO and there is space in the
    // data ring buffer.
    struct transport_frame *ret = 0;
    if (self->transport_fifo.n_frames < TRANSPORT_FIFO_MAX_FRAMES) {
        // Is there space in the ring buffer for the frame payload?
        if(self->transport_fifo.n_ring_buffer_bytes <= TRANSPORT_FIFO_MAX_FRAME_DATA - data_size) {
            self->transport_fifo.n_frames++;
            if (self->transport_fifo.n_frames > self->transport_fifo.n_frames_max) {
                // High-water mark of FIFO (for diagnostic purposes)
                self->transport_fifo.n_frames_max = self->transport_fifo.n_frames;
            }
            // Create FIFO entry
            ret = &(self->transport_fifo.frames[self->transport_fifo.tail_idx]);
            ret->payload_offset = self->transport_fifo.ring_buffer_tail_offset;

            // Claim ring buffer space
            self->transport_fifo.n_ring_buffer_bytes += data_size;
            if(self->transport_fifo.n_ring_buffer_bytes > self->transport_fifo.n_ring_buffer_bytes_max) {
                // High-water mark of ring buffer usage (for diagnostic purposes)
                self->transport_fifo.n_ring_buffer_bytes_max = self->transport_fifo.n_ring_buffer_bytes;
            }
            self->transport_fifo.ring_buffer_tail_offset += data_size;
            self->transport_fifo.ring_buffer_tail_offset &= TRANSPORT_FIFO_SIZE_FRAME_DATA_MASK;

            // Claim FIFO space
            self->transport_fifo.tail_idx++;
            self->transport_fifo.tail_idx &= TRANSPORT_FIFO_SIZE_FRAMES_MASK;
        }
        else {
            min_debug_print("No FIFO payload space: data_size=%d, n_ring_buffer_bytes=%d\n", data_size, self->transport_fifo.n_ring_buffer_bytes);
        }
    }
    else {
        min_debug_print("No FIFO frame slots\n");
    }
    return ret;
}

// Return the nth frame in the FIFO
static struct transport_frame *transport_fifo_get(struct min_context *self, uint8_t n)
{
    uint8_t idx = self->transport_fifo.head_idx;
    return &self->transport_fifo.frames[(idx + n) & TRANSPORT_FIFO_SIZE_FRAMES_MASK];
}

// Sends the given frame to the serial line
static void transport_fifo_send(struct min_context *self, struct transport_frame *frame)
{
    min_debug_print("transport_fifo_send: min_id=%d, seq=%d, payload_len=%d\n", frame->min_id, frame->seq, frame->payload_len);
    on_wire_bytes(self, frame->min_id | (uint8_t)0x80U, frame->seq, payloads_ring_buffer, frame->payload_offset, TRANSPORT_FIFO_SIZE_FRAME_DATA_MASK, frame->payload_len);
    frame->last_sent_time_ms = now;
}

// We don't queue an ACK frame - we send it straight away (if there's space to do so)
static void send_ack(struct min_context *self)
{
    // In the embedded end we don't reassemble out-of-order frames and so never ask for retransmits. Payload is
    // always the same as the sequence number.
    min_debug_print("send ACK: seq=%d\n", self->transport_fifo.rn);
    if(ON_WIRE_SIZE(0) <= min_tx_space(self->port)) {
        on_wire_bytes(self, ACK, self->transport_fifo.rn, &self->transport_fifo.rn, 0, 0xffU, 1U);
        self->transport_fifo.last_sent_ack_time_ms = now;
    }
}

// We don't queue an RESET frame - we send it straight away (if there's space to do so)
static void send_reset(struct min_context *self)
{
    min_debug_print("send RESET\n");
    if(ON_WIRE_SIZE(0) <= min_tx_space(self->port)) {
        on_wire_bytes(self, RESET, 0, 0, 0, 0, 0);
    }
}

static void transport_fifo_reset(struct min_context *self)
{
    // Clear down the transmission FIFO queue
    self->transport_fifo.n_frames = 0;
    self->transport_fifo.head_idx = 0;
    self->transport_fifo.tail_idx = 0;
    self->transport_fifo.n_ring_buffer_bytes = 0;
    self->transport_fifo.ring_buffer_tail_offset = 0;
    self->transport_fifo.sn_max = 0;
    self->transport_fifo.sn_min = 0;
    self->transport_fifo.rn = 0;

    // Reset the timers
    self->transport_fifo.last_received_anything_ms = now;
    self->transport_fifo.last_sent_ack_time_ms = now;
    self->transport_fifo.last_received_frame_ms = 0;
}

void min_transport_reset(struct min_context *self, bool inform_other_side)
{
    if (inform_other_side) {
        // Tell the other end we have gone away
        send_reset(self);
    }

    // Throw our frames away
    transport_fifo_reset(self);
}

// Queues a MIN ID / payload frame into the outgoing FIFO
// API call.
// Returns true if the frame was queued OK.
bool min_queue_frame(struct min_context *self, uint8_t min_id, uint8_t *payload, uint8_t payload_len)
{
    struct transport_frame *frame = transport_fifo_push(self, payload_len); // Claim a FIFO slot, reserve space for payload

    // We are just queueing here: the poll() function puts the frame into the window and on to the wire
    if(frame != 0) {
        // Copy frame details into frame slot, copy payload into ring buffer
        frame->min_id = min_id & (uint8_t)0x3fU;
        frame->payload_len = payload_len;

        uint16_t payload_offset = frame->payload_offset;
        for(uint32_t i = 0; i < payload_len; i++) {
            payloads_ring_buffer[payload_offset] = payload[i];
            payload_offset++;
            payload_offset &= TRANSPORT_FIFO_SIZE_FRAME_DATA_MASK;
        }
        min_debug_print("Queued ID=%d, len=%d\n", min_id, payload_len);
        return true;
    }
    else {
        self->transport_fifo.dropped_frames++;
        return false;
    }
}

bool min_queue_has_space_for_frame(struct min_context *self, uint8_t payload_len) {
    return self->transport_fifo.n_frames < TRANSPORT_FIFO_MAX_FRAMES &&
           self->transport_fifo.n_ring_buffer_bytes <= TRANSPORT_FIFO_MAX_FRAME_DATA - payload_len;
}

// Finds the frame in the window that was sent least recently
static struct transport_frame *find_retransmit_frame(struct min_context *self)
{
    uint8_t window_size = self->transport_fifo.sn_max - self->transport_fifo.sn_min;

#ifdef ASSERTION_CHECKS
    assert(window_size > 0);
    assert(window_size <= self->transport_fifo.nframes);
#endif

    // Start with the head of the queue and call this the oldest
    struct transport_frame *oldest_frame = &self->transport_fifo.frames[self->transport_fifo.head_idx];
    uint32_t oldest_elapsed_time = now - oldest_frame->last_sent_time_ms;

    uint8_t idx = self->transport_fifo.head_idx;
    for(uint8_t i = 0; i < window_size; i++) {
        uint32_t elapsed = now - self->transport_fifo.frames[idx].last_sent_time_ms;
        if(elapsed > oldest_elapsed_time) { // Strictly older only; otherwise the earlier frame is deemed the older
            oldest_elapsed_time = elapsed;
            oldest_frame = &self->transport_fifo.frames[idx];
        }
        idx++;
        idx &= TRANSPORT_FIFO_SIZE_FRAMES_MASK;
    }

    return oldest_frame;
}
#endif // TRANSPORT_PROTOCOL

// This runs the receiving half of the transport protocol, acknowledging frames received, discarding
// duplicates received, and handling RESET requests.
static void valid_frame_received(struct min_context *self)
{
    uint8_t id_control = self->rx_frame_id_control;
    uint8_t *payload = self->rx_frame_payload_buf;
    uint8_t payload_len = self->rx_control;

#ifdef TRANSPORT_PROTOCOL
    uint8_t seq = self->rx_frame_seq;
    uint8_t num_acked;
    uint8_t num_nacked;
    uint8_t num_in_window;

    // When we receive anything we know the other end is still active and won't shut down
    self->transport_fifo.last_received_anything_ms = now;

    switch(id_control) {
        case ACK:
            // If we get an ACK then we remove all the acknowledged frames with seq < rn
            // The payload byte specifies the number of NACKed frames: how many we want retransmitted because
            // they have gone missing.
            // But we need to make sure we don't accidentally ACK too many because of a stale ACK from an old session
            num_acked = seq - self->transport_fifo.sn_min;
            num_nacked = payload[0] - seq;
            num_in_window = self->transport_fifo.sn_max - self->transport_fifo.sn_min;

            if(num_acked <= num_in_window) {
                self->transport_fifo.sn_min = seq;
#ifdef ASSERTION_CHECKING
                assert(self->transport_fifo.n_frames >= num_in_window);
                assert(num_in_window <= TRANSPORT_MAX_WINDOW_SIZE);
                assert(num_nacked <= TRANSPORT_MAX_WINDOW_SIZE);
#endif
                // Now pop off all the frames up to (but not including) rn
                // The ACK contains Rn; all frames before Rn are ACKed and can be removed from the window
                min_debug_print("Received ACK seq=%d, num_acked=%d, num_nacked=%d\n", seq, num_acked, num_nacked);
                for(uint8_t i = 0; i < num_acked; i++) {
                    transport_fifo_pop(self);
                }
                uint8_t idx = self->transport_fifo.head_idx;
                // Now retransmit the number of frames that were requested
                for(uint8_t i = 0; i < num_nacked; i++) {
                    struct transport_frame *retransmit_frame = &self->transport_fifo.frames[idx];
                    transport_fifo_send(self, retransmit_frame);
                    idx++;
                    idx &= TRANSPORT_FIFO_SIZE_FRAMES_MASK;
                }
            }
            else {
                min_debug_print("Received spurious ACK seq=%d\n", seq);
                self->transport_fifo.spurious_acks++;
            }
            break;
        case RESET:
            // If we get a RESET demand then we reset the transport protocol (empty the FIFO, reset the
            // sequence numbers, etc.)
            // We don't send anything, we just do it. The other end can send frames to see if this end is
            // alive (pings, etc.) or just wait to get application frames.
            self->transport_fifo.resets_received++;
            transport_fifo_reset(self);
            break;
        default:
            if (id_control & 0x80U) {
                // Incoming application frames

                // Reset the activity time (an idle connection will be stalled)
                self->transport_fifo.last_received_frame_ms = now;

                if (seq == self->transport_fifo.rn) {
                    // Accept this frame as matching the sequence number we were looking for

                    // Now looking for the next one in the sequence
                    self->transport_fifo.rn++;

                    // Always send an ACK back for the frame we received
                    // ACKs are short (should be about 9 microseconds to send on the wire) and
                    // this will cut the latency down.
                    // We also periodically send an ACK in case the ACK was lost, and in any case
                    // frames are re-sent.
                    send_ack(self);

                    // Now ready to pass this up to the application handlers

                    // Pass frame up to application handler to deal with
                    min_debug_print("Incoming app frame seq=%d, id=%d, payload len=%d\n", seq, id_control & (uint8_t)0x3fU, payload_len);
                    min_application_handler(id_control & (uint8_t)0x3fU, payload, payload_len, self->port);
                } else {
                    // Discard this frame because we aren't looking for it: it's either a dupe because it was
                    // retransmitted when our ACK didn't get through in time, or else it's further on in the
                    // sequence and others got dropped.
                    self->transport_fifo.sequence_mismatch_drop++;
                }
            }
            else {
                // Not a transport frame
                min_application_handler(id_control & (uint8_t)0x3fU, payload, payload_len, self->port);
            }
            break;
    }
#else // TRANSPORT_PROTOCOL
    min_application_handler(id_control & (uint8_t)0x3fU, payload, payload_len, self->port);
#endif // TRANSPORT_PROTOCOL
}

static void rx_byte(struct min_context *self, uint8_t byte)
{
    // Regardless of state, three header bytes means "start of frame" and
    // should reset the frame buffer and be ready to receive frame data
    //
    // Two in a row in over the frame means to expect a stuff byte.
    uint32_t crc;

    if(self->rx_header_bytes_seen == 2) {
        self->rx_header_bytes_seen = 0;
        if(byte == HEADER_BYTE) {
            self->rx_frame_state = RECEIVING_ID_CONTROL;
            return;
        }
        if(byte == STUFF_BYTE) {
            /* Discard this byte; carry on receiving on the next character */
            return;
        }
        else {
            /* Something has gone wrong, give up on this frame and look for header again */
            self->rx_frame_state = SEARCHING_FOR_SOF;
            return;
        }
    }

    if(byte == HEADER_BYTE) {
        self->rx_header_bytes_seen++;
    }
    else {
        self->rx_header_bytes_seen = 0;
    }

    switch(self->rx_frame_state) {
        case SEARCHING_FOR_SOF:
            break;
        case RECEIVING_ID_CONTROL:
            self->rx_frame_id_control = byte;
            self->rx_frame_payload_bytes = 0;
            crc32_init_context(&self->rx_checksum);
            crc32_step(&self->rx_checksum, byte);
            if(byte & 0x80U) {
#ifdef TRANSPORT_PROTOCOL
                self->rx_frame_state = RECEIVING_SEQ;
#else
                // If there is no transport support compiled in then all transport frames are ignored
                self->rx_frame_state = SEARCHING_FOR_SOF;
#endif // TRANSPORT_PROTOCOL
            }
            else {
                self->rx_frame_seq = 0;
                self->rx_frame_state = RECEIVING_LENGTH;
            }
            break;
        case RECEIVING_SEQ:
            self->rx_frame_seq = byte;
            crc32_step(&self->rx_checksum, byte);
            self->rx_frame_state = RECEIVING_LENGTH;
            break;
        case RECEIVING_LENGTH:
            self->rx_frame_length = byte;
            self->rx_control = byte;
            crc32_step(&self->rx_checksum, byte);
            if(self->rx_frame_length > 0) {
                // Can reduce the RAM size by compiling limits to frame sizes
                if(self->rx_frame_length <= MAX_PAYLOAD) {
                    self->rx_frame_state = RECEIVING_PAYLOAD;
                }
                else {
                    // Frame dropped because it's longer than any frame we can buffer
                    self->rx_frame_state = SEARCHING_FOR_SOF;
                }
            }
            else {
                self->rx_frame_state = RECEIVING_CHECKSUM_3;
            }
            break;
        case RECEIVING_PAYLOAD:
            self->rx_frame_payload_buf[self->rx_frame_payload_bytes++] = byte;
            crc32_step(&self->rx_checksum, byte);
            if(--self->rx_frame_length == 0) {
                self->rx_frame_state = RECEIVING_CHECKSUM_3;
            }
            break;
        case RECEIVING_CHECKSUM_3:
            self->rx_frame_checksum = ((uint32_t)byte) << 24;
            self->rx_frame_state = RECEIVING_CHECKSUM_2;
            break;
        case RECEIVING_CHECKSUM_2:
            self->rx_frame_checksum |= ((uint32_t)byte) << 16;
            self->rx_frame_state = RECEIVING_CHECKSUM_1;
            break;
        case RECEIVING_CHECKSUM_1:
            self->rx_frame_checksum |= ((uint32_t)byte) << 8;
            self->rx_frame_state = RECEIVING_CHECKSUM_0;
            break;
        case RECEIVING_CHECKSUM_0:
            self->rx_frame_checksum |= byte;
            crc = crc32_finalize(&self->rx_checksum);
            if(self->rx_frame_checksum != crc) {
                // Frame fails the checksum and so is dropped
                self->rx_frame_state = SEARCHING_FOR_SOF;
            }
            else {
                // Checksum passes, go on to check for the end-of-frame marker
                self->rx_frame_state = RECEIVING_EOF;
            }
            break;
        case RECEIVING_EOF:
            if(byte == 0x55u) {
                // Frame received OK, pass up data to handler
                valid_frame_received(self);
            }
            // else discard
            // Look for next frame */
            self->rx_frame_state = SEARCHING_FOR_SOF;
            break;
        default:
            // Should never get here but in case we do then reset to a safe state
            self->rx_frame_state = SEARCHING_FOR_SOF;
            break;
    }
}

// API call: sends received bytes into a MIN context and runs the transport timeouts
void min_poll(struct min_context *self, uint8_t *buf, uint32_t buf_len)
{
    for(uint32_t i = 0; i < buf_len; i++) {
        rx_byte(self, buf[i]);
    }

#ifdef TRANSPORT_PROTOCOL
    uint8_t window_size;

    now = min_time_ms();

    bool remote_connected = (now - self->transport_fifo.last_received_anything_ms < TRANSPORT_IDLE_TIMEOUT_MS);
    bool remote_active = (now - self->transport_fifo.last_received_frame_ms < TRANSPORT_IDLE_TIMEOUT_MS);

    // This sends one new frame or resends one old frame
    window_size = self->transport_fifo.sn_max - self->transport_fifo.sn_min; // Window size
    if((window_size < TRANSPORT_MAX_WINDOW_SIZE) && (self->transport_fifo.n_frames > window_size)) {
        // There are new frames we can send; but don't even bother if there's no buffer space for them
        struct transport_frame *frame = transport_fifo_get(self, window_size);
        if(ON_WIRE_SIZE(frame->payload_len) <= min_tx_space(self->port)) {
            frame->seq = self->transport_fifo.sn_max;
            transport_fifo_send(self, frame);

            // Move window on
            self->transport_fifo.sn_max++;
        }
    }
    else {
        // Sender cannot send new frames so resend old ones (if there's anyone there)
        if((window_size > 0) && remote_connected) {
            // There are unacknowledged frames. Can re-send an old frame. Pick the least recently sent one.
            struct transport_frame *oldest_frame = find_retransmit_frame(self);
            if(now - oldest_frame->last_sent_time_ms >= TRANSPORT_FRAME_RETRANSMIT_TIMEOUT_MS) {
                // Resending oldest frame if there's a chance there's enough space to send it
                if(ON_WIRE_SIZE(oldest_frame->payload_len) <= min_tx_space(self->port)) {
                    transport_fifo_send(self, oldest_frame);
                }
            }
        }
    }

#ifndef DISABLE_TRANSPORT_ACK_RETRANSMIT
    // Periodically transmit the ACK with the rn value, unless the line has gone idle
    if(now - self->transport_fifo.last_sent_ack_time_ms > TRANSPORT_ACK_RETRANSMIT_TIMEOUT_MS) {
        if(remote_active) {
            send_ack(self);
        }
    }
#endif // DISABLE_TRANSPORT_ACK_RETRANSMIT
#endif // TRANSPORT_PROTOCOL
}

void min_init_context(struct min_context *self, uint8_t port)
{
    // Initialize context
    self->rx_header_bytes_seen = 0;
    self->rx_frame_state = SEARCHING_FOR_SOF;
    self->port = port;

#ifdef TRANSPORT_PROTOCOL
    // Counters for diagnosis purposes
    self->transport_fifo.spurious_acks = 0;
    self->transport_fifo.sequence_mismatch_drop = 0;
    self->transport_fifo.dropped_frames = 0;
    self->transport_fifo.resets_received = 0;
    self->transport_fifo.n_ring_buffer_bytes_max = 0;
    self->transport_fifo.n_frames_max = 0;
    transport_fifo_reset(self);
#endif // TRANSPORT_PROTOCOL
}

// Sends an application MIN frame on the wire (do not put into the transport queue)
void min_send_frame(struct min_context *self, uint8_t min_id, uint8_t *payload, uint8_t payload_len)
{
    if((ON_WIRE_SIZE(payload_len) <= min_tx_space(self->port))) {
        on_wire_bytes(self, min_id & (uint8_t) 0x3fU, 0, payload, 0, 0xffffU, payload_len);
    }
}

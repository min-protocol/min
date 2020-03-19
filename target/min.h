// MIN Protocol v2.0.
//
// MIN is a lightweight reliable protocol for exchanging information from a microcontroller (MCU) to a host.
// It is designed to run on an 8-bit MCU but also scale up to more powerful devices. A typical use case is to
// send data from a UART on a small MCU over a UART-USB converter plugged into a PC host. A Python implementation
// of host code is provided (or this code could be compiled for a PC).
//
// MIN supports frames of 0-255 bytes (with a lower limit selectable at compile time to reduce RAM). MIN frames
// have identifier values between 0 and 63.
//
// An optional transport layer T-MIN can be compiled in. This provides sliding window reliable transmission of frames.
//
// Compile options:
//
// -  Define NO_TRANSPORT_PROTOCOL to remove the code and other overheads of dealing with transport frames. Any
//    transport frames sent from the other side are dropped.
//
// -  Define MAX_PAYLOAD if the size of the frames is to be limited. This is particularly useful with the transport
//    protocol where a deep FIFO is wanted but not for large frames.
//
// The API is as follows:
//
// -  min_init_context()
//    A MIN context is a structure allocated by the programmer that stores details of the protocol. This permits
//    the code to be reentrant and multiple serial ports to be used. The port parameter is used in a callback to
//    allow the programmer's serial port drivers to place bytes in the right port. In a typical scenario there will
//    be just one context.
//
// -  min_send_frame()
//    This sends a non-transport frame and will be dropped if the line is noisy.
//
// -  min_queue_frame()
//    This queues a transport frame which will will be retransmitted until the other side receives it correctly.
//
// -  min_poll()
//    This passes in received bytes to the context associated with the source. Note that if the transport protocol
//    is included then this must be called regularly to operate the transport state machine even if there are no
//    incoming bytes.
//
// There are several callbacks: these must be provided by the programmer and are called by the library:
//
// -  min_tx_space()
//    The programmer's serial drivers must return the number of bytes of space available in the sending buffer.
//    This helps cut down on the number of lost frames (and hence improve throughput) if a doomed attempt to transmit a
//    frame can be avoided.
//
// -  min_tx_byte()
//    The programmer's drivers must send a byte on the given port. The implementation of the serial port drivers
//    is in the domain of the programmer: they might be interrupt-based, polled, etc.
//
// -  min_application_handler()
//    This is the callback that provides a MIN frame received on a given port to the application. The programmer
//    should then deal with the frame as part of the application.
//
// -  min_time_ms()
//    This is called to obtain current time in milliseconds. This is used by the MIN transport protocol to drive
//    timeouts and retransmits.


#ifndef MIN_H
#define MIN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef ASSERTION_CHECKING
#include <assert.h>
#endif

#ifndef NO_TRANSPORT_PROTOCOL
#define TRANSPORT_PROTOCOL
#endif

#ifndef MAX_PAYLOAD
#define MAX_PAYLOAD                                 (255U)
#endif

// Powers of two for FIFO management. Default is 16 frames in the FIFO, total of 1024 bytes for frame data
#ifndef TRANSPORT_FIFO_SIZE_FRAMES_BITS
#define TRANSPORT_FIFO_SIZE_FRAMES_BITS             (4U)
#endif
#ifndef TRANSPORT_FIFO_SIZE_FRAME_DATA_BITS
#define TRANSPORT_FIFO_SIZE_FRAME_DATA_BITS         (10U)
#endif

#define TRANSPORT_FIFO_MAX_FRAMES                   (1U << TRANSPORT_FIFO_SIZE_FRAMES_BITS)
#define TRANSPORT_FIFO_MAX_FRAME_DATA               (1U << TRANSPORT_FIFO_SIZE_FRAME_DATA_BITS)

#if (MAX_PAYLOAD > 255)
#error "MIN frame payloads can be no bigger than 255 bytes"
#endif

// Indices into the frames FIFO are uint8_t and so can't have more than 256 frames in a FIFO
#if (TRANSPORT_FIFO_MAX_FRAMES > 256)
#error "Transport FIFO frames cannot exceed 256"
#endif

// Using a 16-bit offset into the frame data FIFO so it has to be addressable within 64Kbytes
#if (TRANSPORT_FIFO_MAX_FRAME_DATA > 65536)
#error "Transport FIFO data allocated cannot exceed 64Kbytes"
#endif

struct crc32_context {
    uint32_t crc;
};

#ifdef TRANSPORT_PROTOCOL

struct transport_frame {
    uint32_t last_sent_time_ms;                     // When frame was last sent (used for re-send timeouts)
    uint16_t payload_offset;                        // Where in the ring buffer the payload is
    uint8_t payload_len;                            // How big the payload is
    uint8_t min_id;                                 // ID of frame
    uint8_t seq;                                    // Sequence number of frame
};

struct transport_fifo {
    struct transport_frame frames[TRANSPORT_FIFO_MAX_FRAMES];
    uint32_t last_sent_ack_time_ms;
    uint32_t last_received_anything_ms;
    uint32_t last_received_frame_ms;
    uint32_t dropped_frames;                        // Diagnostic counters
    uint32_t spurious_acks;
    uint32_t sequence_mismatch_drop;
    uint32_t resets_received;
    uint16_t n_ring_buffer_bytes;                   // Number of bytes used in the payload ring buffer
    uint16_t n_ring_buffer_bytes_max;               // Largest number of bytes ever used
    uint16_t ring_buffer_tail_offset;               // Tail of the payload ring buffer
    uint8_t n_frames;                               // Number of frames in the FIFO
    uint8_t n_frames_max;                           // Larger number of frames in the FIFO
    uint8_t head_idx;                               // Where frames are taken from in the FIFO
    uint8_t tail_idx;                               // Where new frames are added
    uint8_t sn_min;                                 // Sequence numbers for transport protocol
    uint8_t sn_max;
    uint8_t rn;
};
#endif

struct min_context {
#ifdef TRANSPORT_PROTOCOL
    struct transport_fifo transport_fifo;           // T-MIN queue of outgoing frames
#endif
    uint8_t rx_frame_payload_buf[MAX_PAYLOAD];      // Payload received so far
    uint32_t rx_frame_checksum;                     // Checksum received over the wire
    struct crc32_context rx_checksum;               // Calculated checksum for receiving frame
    struct crc32_context tx_checksum;               // Calculated checksum for sending frame
    uint8_t rx_header_bytes_seen;                   // Countdown of header bytes to reset state
    uint8_t rx_frame_state;                         // State of receiver
    uint8_t rx_frame_payload_bytes;                 // Length of payload received so far
    uint8_t rx_frame_id_control;                    // ID and control bit of frame being received
    uint8_t rx_frame_seq;                           // Sequence number of frame being received
    uint8_t rx_frame_length;                        // Length of frame
    uint8_t rx_control;                             // Control byte
    uint8_t tx_header_byte_countdown;               // Count out the header bytes
    uint8_t port;                                   // Number of the port associated with the context
};

#ifdef TRANSPORT_PROTOCOL
// Queue a MIN frame in the transport queue
bool min_queue_frame(struct min_context *self, uint8_t min_id, uint8_t *payload, uint8_t payload_len);

// Determine if MIN has space to queue a transport frame
bool min_queue_has_space_for_frame(struct min_context *self, uint8_t payload_len);
#endif

// Send a non-transport frame MIN frame
void min_send_frame(struct min_context *self, uint8_t min_id, uint8_t *payload, uint8_t payload_len);

// Must be regularly called, with the received bytes since the last call.
// NB: if the transport protocol is being used then even if there are no bytes
// this call must still be made in order to drive the state machine for retransmits.
void min_poll(struct min_context *self, uint8_t *buf, uint32_t buf_len);

// Reset the state machine and (optionally) tell the other side that we have done so
void min_transport_reset(struct min_context *self, bool inform_other_side);

// CALLBACK. Handle incoming MIN frame
void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port);

#ifdef TRANSPORT_PROTOCOL
// CALLBACK. Must return current time in milliseconds.
// Typically a tick timer interrupt will increment a 32-bit variable every 1ms (e.g. SysTick on Cortex M ARM devices).
uint32_t min_time_ms(void);
#endif

// CALLBACK. Must return current buffer space in the given port. Used to check that a frame can be
// queued.
uint16_t min_tx_space(uint8_t port);

// CALLBACK. Send a byte on the given line.
void min_tx_byte(uint8_t port, uint8_t byte);

// CALLBACK. Indcates when frame transmission is finished; useful for buffering bytes into a single serial call.
void min_tx_start(uint8_t port);
void min_tx_finished(uint8_t port);

// Initialize a MIN context ready for receiving bytes from a serial link
// (Can have multiple MIN contexts)
void min_init_context(struct min_context *self, uint8_t port);

#ifdef MIN_DEBUG_PRINTING
// Debug print
void min_debug_print(const char *msg, ...);
#else
#define min_debug_print(...)
#endif

#endif //MIN_H

/* codegen.c */

/* AUTOMATICALLY GENERATED C CODE; DO NOT EDIT */

#include "gen.h"

/* Network ordering conversion functions (compiler should inline) */
static void encode_32(uint32_t data, uint8_t buf[])
{
    buf[0] = (uint8_t)((data & 0xff000000UL) >> 24);
    buf[1] = (uint8_t)((data & 0x00ff0000UL) >> 16);
    buf[2] = (uint8_t)((data & 0x0000ff00UL) >> 8);
    buf[3] = (uint8_t)(data & 0x000000ffUL);
}

static void encode_16(uint32_t data, uint8_t buf[])
{
    buf[0] = (uint8_t)((data & 0x0000ff00UL) >> 8);
    buf[1] = (uint8_t)(data & 0x000000ffUL);
}

static uint32_t decode_32(uint8_t buf[])
{
    uint32_t res;
    res = ((uint32_t)(buf[0]) << 24) | ((uint32_t)(buf[1]) << 16) | ((uint32_t)(buf[2]) << 8) | (uint32_t)(buf[3]);
    return res;
}

static uint16_t decode_16(uint8_t buf[])
{
    uint16_t res;
    res = ((uint16_t)(buf[0]) << 8) | (uint16_t)(buf[1]);
    return res;
}

/* Macros for packing and unpacking frame buffers */
#define DECLARE_BUF(size) uint8_t m_buf[(size)]; uint8_t m_control = (size); uint8_t m_cursor = 0

#define PACK8(v) (m_buf[m_cursor] = (v), m_cursor++)
#define PACK16(v) (encode_16((v), m_buf + m_cursor), m_cursor += 2U)
#define PACK32(v) (encode_32((v), m_buf + m_cursor), m_cursor += 4U)

#define SEND_FRAME(id) (min_tx_frame((id), m_buf, m_control))

#define DECLARE_UNPACK() uint8_t m_cursor = 0

#define UNPACK8(v) ((v) = (int8_t)m_buf[m_cursor], m_cursor++)
#define UNPACK16(v) ((v) = (int16_t)decode_16(m_buf + m_cursor), m_cursor += 2U)
#define UNPACK32(v) ((v) = (int32_t)decode_32(m_buf + m_cursor), m_cursor += 4U
#define UNPACKU8(v) ((v) = m_buf[m_cursor], m_cursor++)
#define UNPACKU16(v) ((v) = decode_16(m_buf + m_cursor), m_cursor += 2U)
#define UNPACKU32(v) ((v) = decode_32(m_buf + m_cursor), m_cursor += 4U)

/* Update bit bytes for frames */
{% for frame in frames %}
    {% for n in range(0, frame.update_bit_bytes) %}
uint8_t {{ frame.update_byte_name_prefix() }}{{ n }};
    {% endfor %}
{% endfor %}

/* Signal frame queueing functions */
{% for frame in frames %}
    {% if frame.output and not frame.raw %}
void min_queue_frame_{{ frame.handle }}(void)
{
    DECLARE_BUF({{ frame.frame_size }}U);
        {% for n in range(0, frame.update_bit_bytes) %}
    PACK8({{ frame.update_byte_name_prefix() }}{{ n }});
    {{ frame.update_byte_name_prefix() }}{{ n }} = 0;
        {% endfor %}
        {% for signal in frame.signals %}
    PACK{{ signal.size * 8 }}({{ signal.variable_name() }});
        {% endfor %}
    SEND_FRAME({{ frame.min_id }}U);
}
    {% endif %}
{% endfor %}

/* Functions for sending raw frames */
{% for frame in frames %}
    {% if frame.output and frame.raw %}
void min_queue_frame_{{ frame.handle }}(uint8_t buf[], uint8_t control)
{
        {% if frame.period_guarded() %}
    if({{ frame.period_counter_name()}} == 0) {
        min_tx_frame({{ frame.min_id }}, buf, control);
        {{ frame.period_counter_name() }} = {{ frame.get_period() }};
    }
        {% else %}
    min_tx_frame({{ frame.min_id }}, buf, control);
        {% endif %}
}
    {% endif %}
{% endfor %}

/* Functions for unpacking raw frames do not appear here: they are callbacks into user code */

/* Frame sending period guard counters for raw and forced transmit frames */
{% for frame in frames %}
    {% if frame.output and frame.period_guarded() %}
uint8_t {{ frame.period_counter_name() }};
    {% endif %}
{% endfor %}


/* Frame sending period counters for normal periodic frames */
{% for frame in frames %}
    {% if frame.output and frame.signal_frame() %}
uint8_t {{ frame.period_counter_name() }};
    {% endif %}
{% endfor %}

/* Signal variables declarations (both input and output) */

{% for signal in signals %}
{{ signal.c_type }} {{ signal.variable_name() }};
{% endfor %}

/* Functions for sending force transmit signals */

{% for signal in signals %}
    {% if signal == signal.frame.forced_transmit_signal %}
void set_{{ signal.handle }}({{ signal.ctype }} n)
{
    {{ signal.variable_name() }} = n;
        {% if signal.update_bit %}
    {{ signal.update_byte_name() }} |= {{ signal.update_bit_mask() }}U;
        {% endif %}
        {% if signal.frame.period_guarded() %}
    if({{ signal.frame.period_counter_name()}} == 0) {
        min_queue_frame_{{ signal.frame.handle }}();
        {{ signal.frame_period_counter_name() }} = {{ signal.frame.get_period() }};
    }
        {% else %}
    min_queue_frame_{{ signal.frame.handle }}();
        {% endif %}
}
    {% endif %}
{% endfor %}

/* Functions for unpacking signal frames */

{% for frame in frames %}
    {% if frame.input and not frame.raw %}
static void min_unpack_frame_{{ frame.handle }}(uint8_t m_buf[])
{
    DECLARE_UNPACK();
        {% for n in range(0, frame.update_bit_bytes) %}
    UNPACKU8({{ frame.update_byte_name_prefix() }}{{ n }});
        {% endfor %}
        {% for signal in frame.signals %}
    UNPACK{{ signal.signed_char() }}{{ signal.size * 8 }}({{ signal.variable_name() }});
        {% endfor %}
}
    {% endif %}
{% endfor %}

void min_input(void)
{
    /* Handle all the outstanding characters in the input buffer */
    while(uart_receive_ready()) {
        uint8_t byte;
        uart_receive(&byte, 1U);
        min_rx_byte(byte);
    }
}

void min_frame_received(uint8_t buf[], uint8_t control, uint8_t id)
{
    switch(id) {
{% for frame in frames %}
    {% if frame.input %}
        case {{ frame.min_id }}U:
            min_unpack_frame_{{ frame.handle }}(buf);
            break;
    {% endif %}
{% endfor %}
    }
}

void min_initialize(void)
{
{% for frame in frames %}
    {% if frame.output %}
        {% if frame.period_guarded() %}
    {{ frame.period_counter_name() }} == 0;
        {% endif %}
        {% if frame.signal_frame() %}
    {{ frame.period_counter_name() }} == {{ frame.get_offset() + 1}}U;
        {% endif %}
    {% endif %}
{% endfor %}
}

/* Handle counters for period guarded frames (raw and force transmit) and also periodic signal frames */
void min_output(void)
{
{% for frame in frames %}
    {% if frame.output %}
        {% if frame.period_guarded() %}
    if({{ frame.period_counter_name() }}) {
        {{ frame.period_counter_name() }}--;
    }
        {% endif %}
        {% if frame.signal_frame() %}
    if(--{{ frame.period_counter_name() }} == 0) {
        {{ frame.period_counter_name() }} = {{ frame.get_period() }}U;
        min_queue_frame_{{ frame.handle }}();
    }
        {% endif %}
    {% endif %}
{% endfor %}
}

/* codegen.h */

/* AUTOMATICALLY GENERATED CODE: DO NOT EDIT DIRECTLY */

#include "../firmware/min/min.h"
#include <stdint.h>

/* Update bit bytes for frames */
{% for frame in frames %}
    {% for n in range(0, frame.update_bit_bytes) %}
extern uint8_t {{ frame.update_byte_name_prefix() }}{{ n }};
    {% endfor %}
{% endfor %}

/* Frame sending period counters */
{% for frame in frames %}
    {% if frame.output and frame.period_guarded() %}
extern uint8_t {{ frame.period_counter_name() }};
    {% endif %}
{% endfor %}

/* Prototypes for raw frame queueing functions */
{% for frame in frames %}
    {% if frame.output and frame.raw %}
void min_queue_frame_{{ frame.handle }}(uint8_t buf[], uint8_t control); /* Raw frame */
    {% endif %}
{% endfor %}

/* Prototypes for unpacking raw frames; these are callbacks to user-provided functions */
{% for frame in frames %}
    {% if frame.input and frame.raw %}
void min_unpack_frame_{{ frame.handle }}(uint8_t buf[], uint8_t control); /* Raw frame; user-provided function */
    {% endif %}
{% endfor %}

/* Signal variables declarations (both input and output) */
{% for signal in signals %}
extern {{ signal.c_type }} {{ signal.variable_name() }};
{% endfor %}

/* Prototype for sending force transmit signals */
{% for signal in signals %}
    {% if signal == signal.frame.forced_transmit_signal %}
void set_{{ signal.handle }}({{ signal.ctype }} n);
    {% endif %}
{% endfor %}

/* Inline macros for sending normal signals */
{% for signal in signals %}
    {% if signal.frame.output and signal != signal.frame.forced_transmit_signal %}
#define set_{{ signal.handle }}(n) ({{ signal.variable_name() }}) = (n),\
        {% if signal.update_bit %}
            {{ signal.update_byte_name() }} |= {{ signal.update_bit_mask() }}U)
        {% else %}
            0)
        {% endif %}
    {% endif %}
{% endfor %}

/* Received signal macros */
{% for signal in signals %}
    {% if signal.frame.input %}
#define get_{{ signal.handle }}()       ({{ signal.variable_name() }})
    {% endif %}
{% endfor %}

/* Update bit testing for received signals */
{% for signal in signals %}
    {% if signal.update_bit and signal.frame.input %}
#define updated_{{ signal.handle }}()   ({{ signal.update_byte_name() }} & {{ signal.update_bit_mask() }})
    {% endif %}
{% endfor %}

/* Clear update bit of received signal */
{% for signal in signals %}
    {% if signal.update_bit and signal.frame.input %}
#define clear_updated_{{ signal.handle }}()    ({{ signal.update_byte_name() }} &= ~{{ signal.update_bit_mask() }})
    {% endif %}
{% endfor %}

void min_input(void);
void min_output(void);
void min_initialize(void);

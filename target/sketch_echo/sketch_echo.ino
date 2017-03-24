//
// Echo server the Arduino M0 Pro (based on Atmel SAMD21)


#include <stdarg.h>
#define DEBUG_PRINTING

// Debug printing callback
void min_debug_print(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 256, fmt, args);
    va_end (args);
    Serial.print(buf);
}

#include "/Users/kentindell/github/min/target/min.h"
#include "/Users/kentindell/github/min/target/min.c"

struct min_context min_ctx;

////////////////////////////////// CALLBACKS ///////////////////////////////////

uint16_t min_tx_space(uint8_t port)
{
  uint16_t n = SerialUSB.availableForWrite();

  // This is a lie but we will handle the consequences
  return 512U;
}

// Send a character on the designated port.
void min_tx_byte(uint8_t port, uint8_t byte)
{
  // If there's no space, spin waiting for space
  uint32_t n;
  do {
    n = SerialUSB.write(&byte, 1U);
  }
  while(n == 0);
}

// Tell MIN the current time in milliseconds.
uint32_t min_time_ms(void)
{
  return millis();
}

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
  // In this simple example application we just echo the frame back when we get one
  bool result = min_queue_frame(&min_ctx, min_id, min_payload, len_payload);
  if(!result) {
    Serial.println("Queue failed");
  }
}

void setup() {
  Serial.begin(250000);
  SerialUSB.begin(250000);
  while(!Serial) {
    ; // Wait for serial port
  }
  while(!SerialUSB) {
    ;
  }
  min_init_context(&min_ctx, 0);
}

void loop() {
  char buf[32];
  size_t buf_len;

  if(SerialUSB.available() > 0) {
    buf_len = SerialUSB.readBytes(buf, 32U);
  }
  else {
    buf_len = 0;
  }
  min_poll(&min_ctx, (uint8_t *)buf, (uint8_t)buf_len);
}



#ifndef RINGBUF_H
#define RINGBUF_H
/*
 * Byte ring buffer, single-producer / single-consumer (SPSC).
 *
 * Safe WITHOUT locks when exactly one context writes and exactly one context
 * reads (e.g. UART ISR writes, main loop reads). Not safe with two writers.
 * Capacity must be a power of two. No HAL includes: compiles on the host.
 */
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t *buf;
    uint32_t mask;            /* capacity - 1 */
    volatile uint32_t head;   /* written only by producer */
    volatile uint32_t tail;   /* written only by consumer */
    volatile uint32_t dropped;/* bytes rejected because the buffer was full */
} ringbuf_t;

/* storage_size must be a power of two. Returns 0 on success, -1 otherwise. */
int      ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint32_t storage_size);
uint32_t ringbuf_count(const ringbuf_t *rb);   /* bytes available to read */
uint32_t ringbuf_space(const ringbuf_t *rb);   /* bytes free to write     */
/* Writes up to len bytes; returns bytes actually written (rest are dropped). */
uint32_t ringbuf_write(ringbuf_t *rb, const uint8_t *data, uint32_t len);
/* Copies up to len bytes out WITHOUT consuming them. Returns bytes copied. */
uint32_t ringbuf_peek(const ringbuf_t *rb, uint8_t *out, uint32_t len);
/* Consumes up to len bytes. Returns bytes consumed. */
uint32_t ringbuf_drop(ringbuf_t *rb, uint32_t len);
/* peek + drop in one call. */
uint32_t ringbuf_read(ringbuf_t *rb, uint8_t *out, uint32_t len);

#endif

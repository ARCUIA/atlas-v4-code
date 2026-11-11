#include "ringbuf.h"

int ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint32_t storage_size)
{
    if (!rb || !storage || storage_size < 2u || (storage_size & (storage_size - 1u)))
        return -1;
    rb->buf = storage;
    rb->mask = storage_size - 1u;
    rb->head = 0;
    rb->tail = 0;
    rb->dropped = 0;
    return 0;
}

uint32_t ringbuf_count(const ringbuf_t *rb)
{
    return (rb->head - rb->tail) & 0xFFFFFFFFu;   /* free-running indices */
}

uint32_t ringbuf_space(const ringbuf_t *rb)
{
    return (rb->mask + 1u) - ringbuf_count(rb);
}

uint32_t ringbuf_write(ringbuf_t *rb, const uint8_t *data, uint32_t len)
{
    uint32_t space = ringbuf_space(rb);
    uint32_t n = (len < space) ? len : space;
    uint32_t head = rb->head;
    for (uint32_t i = 0; i < n; i++)
        rb->buf[(head + i) & rb->mask] = data[i];
    rb->head = head + n;            /* publish AFTER the data is stored */
    if (n < len)
        rb->dropped += (len - n);
    return n;
}

uint32_t ringbuf_peek(const ringbuf_t *rb, uint8_t *out, uint32_t len)
{
    uint32_t avail = ringbuf_count(rb);
    uint32_t n = (len < avail) ? len : avail;
    uint32_t tail = rb->tail;
    for (uint32_t i = 0; i < n; i++)
        out[i] = rb->buf[(tail + i) & rb->mask];
    return n;
}

uint32_t ringbuf_drop(ringbuf_t *rb, uint32_t len)
{
    uint32_t avail = ringbuf_count(rb);
    uint32_t n = (len < avail) ? len : avail;
    rb->tail += n;
    return n;
}

uint32_t ringbuf_read(ringbuf_t *rb, uint8_t *out, uint32_t len)
{
    uint32_t n = ringbuf_peek(rb, out, len);
    rb->tail += n;
    return n;
}

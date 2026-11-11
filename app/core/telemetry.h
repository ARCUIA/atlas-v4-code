#ifndef TELEMETRY_H
#define TELEMETRY_H
/*
 * Decides what goes into the next radio packet. Each "source" adds one or more records to a packet and
 * says how often it wants to be sent. To add telemetry later, write a fill function and register it with
 * tlm_add(); nothing else changes.
 */
#include <stdint.h>
#include <stddef.h>
#include "pkt.h"

#define TLM_MAX_SOURCES 8

typedef int (*tlm_fill_fn)(pkt_builder_t *b, void *ctx);     /* append record(s); return 0 on success */

typedef struct {
    tlm_fill_fn fill; void *ctx;
    uint32_t period_ms, last_ms;
    uint8_t  sent_once;
} tlm_source_t;

typedef struct {
    tlm_source_t src[TLM_MAX_SOURCES];
    uint8_t  n, src_id, have_pkt;
    uint16_t seq;
    uint32_t min_gap_ms, last_pkt_ms;
} tlm_t;

void tlm_init(tlm_t *t, uint8_t src_id, uint32_t min_gap_ms);
int  tlm_add(tlm_t *t, tlm_fill_fn fill, void *ctx, uint32_t period_ms);     /* 0 ok, -1 table full */
/* Returns the packet length if something is due (and min_gap_ms has passed since the last packet), else 0. */
size_t tlm_build(tlm_t *t, uint32_t now_ms, uint8_t *out, size_t cap);

#endif

#include "telemetry.h"

void tlm_init(tlm_t *t, uint8_t src_id, uint32_t min_gap_ms)
{
    t->n = 0; t->src_id = src_id; t->have_pkt = 0; t->seq = 0;
    t->min_gap_ms = min_gap_ms; t->last_pkt_ms = 0;
}

int tlm_add(tlm_t *t, tlm_fill_fn fill, void *ctx, uint32_t period_ms)
{
    if (t->n >= TLM_MAX_SOURCES) return -1;
    tlm_source_t *s = &t->src[t->n++];
    s->fill = fill; s->ctx = ctx; s->period_ms = period_ms; s->last_ms = 0; s->sent_once = 0;
    return 0;
}

/* How overdue a source is. Never-sent sources come first, so nothing starves behind a larger one. */
static uint32_t lateness(const tlm_source_t *s, uint32_t now_ms)
{
    if (!s->sent_once) return 0xFFFFFFFFu;
    uint32_t age = (uint32_t)(now_ms - s->last_ms);
    return age >= s->period_ms ? age - s->period_ms : 0;     /* 0 and "not due" are told apart below */
}

size_t tlm_build(tlm_t *t, uint32_t now_ms, uint8_t *out, size_t cap)
{
    if (t->have_pkt && (uint32_t)(now_ms - t->last_pkt_ms) < t->min_gap_ms) return 0;

    /* Collect the sources that are due and order them most-overdue first (stable for ties). */
    uint8_t order[TLM_MAX_SOURCES], n = 0;
    for (uint8_t i = 0; i < t->n; i++) {
        const tlm_source_t *s = &t->src[i];
        int due = !s->sent_once || (uint32_t)(now_ms - s->last_ms) >= s->period_ms;
        if (due) order[n++] = i;
    }
    for (uint8_t i = 1; i < n; i++) {                       /* insertion sort */
        uint8_t v = order[i];
        uint32_t lv = lateness(&t->src[v], now_ms);
        int j = i - 1;
        while (j >= 0 && lateness(&t->src[order[j]], now_ms) < lv) { order[j + 1] = order[j]; j--; }
        order[j + 1] = v;
    }

    pkt_builder_t b;
    pkt_begin(&b, out, cap, t->src_id, t->seq);
    int added = 0;
    for (uint8_t k = 0; k < n; k++) {
        tlm_source_t *s = &t->src[order[k]];
        size_t before = b.len;
        if (s->fill(&b, s->ctx) == 0 && b.len > before) {
            s->last_ms = now_ms; s->sent_once = 1; added = 1;
        } else {
            b.len = before;                                  /* did not fit: undo, it stays due */
        }
    }
    if (!added) return 0;
    t->seq++; t->last_pkt_ms = now_ms; t->have_pkt = 1;
    return pkt_end(&b);
}

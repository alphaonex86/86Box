/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          The CPU's L1 data cache as RAM, for firmware that keeps its stack
 *          in it before DRAM is up (CR0.CD = NW = 1, 486 and P5).
 *
 *          The chipset's DRAM handlers call it while it is active. INVD
 *          starts recording the lines that read misses fill (where the
 *          chipset asserts KEN#); setting CR0.CD and NW then loads them and
 *          serves them as RAM (SDM vol. 3, "Cache Operating Modes": hits
 *          served, no fills, write hits stay in the line). INVD drops the
 *          lines, WBINVD writes the Modified ones back (filled write-back,
 *          then written), and leaving that mode otherwise keeps what the CPU
 *          saw. With CD clear the cache is coherent, so only tags are kept.
 */
#include <stdint.h>
#include <string.h>
#include <wchar.h>
#include <86box/86box.h>
#include "cpu.h"
#include <86box/timer.h>
#include <86box/dma.h>
#include <86box/cache_l1.h>

#define CR0_CD (1u << 30)
#define CR0_NW (1u << 29)

#define LINE_WB       1 /* filled with WB/WT# high: Exclusive */
#define LINE_MODIFIED 2

const cache_l1_geometry_t cache_l1_486  = { .sets = 256, .ways = 4, .line = 16 };
const cache_l1_geometry_t cache_l1_p54c = { .sets = 128, .ways = 2, .line = 32, .lru = 1 };

static void
cache_l1_clear(cache_l1_t *l1)
{
    memset(l1->tag, 0, sizeof(l1->tag));
}

/* The line that holds addr, or -1; a read miss while recording fills one. */
static int
cache_l1_line(cache_l1_t *l1, uint32_t addr, int read)
{
    const cache_l1_geometry_t *geo   = l1->geo;
    uint32_t                   tag   = (addr & ~(geo->line - 1)) | 1;
    int                        set   = (addr / geo->line) % geo->sets;
    int                        first = set * geo->ways;
    int                        fill;

    if (dma_bm_busy) /* bus master cycles: the CPU's L1 neither fills nor answers */
        return -1;
    for (int way = 0; way < geo->ways; way++)
        if (l1->tag[first + way] == tag) {
            if (geo->lru)
                l1->next[set] = !way;
            return first + way;
        }
    if (read && l1->track) {
        fill = l1->fill ? l1->fill(l1->priv, addr) : CACHE_L1_WT;
        if (fill != CACHE_L1_UNCACHED) {
            int way = l1->next[set]++ % geo->ways;
            if (geo->lru)
                l1->next[set] = !way;
            l1->tag[first + way]   = tag;
            l1->state[first + way] = (fill == CACHE_L1_WB) ? LINE_WB : 0;
        }
    }
    return -1;
}

uint8_t
cache_l1_read(cache_l1_t *l1, uint32_t addr)
{
    int line = cache_l1_line(l1, addr, 1);

    if ((line < 0) || !l1->car)
        return l1->load(l1->priv, addr);
    return l1->data[line * l1->geo->line + (addr & (l1->geo->line - 1))];
}

void
cache_l1_write(cache_l1_t *l1, uint32_t addr, uint8_t val)
{
    int line = cache_l1_line(l1, addr, 0);

    if ((line >= 0) && (l1->state[line] & LINE_WB))
        l1->state[line] |= LINE_MODIFIED;
    if ((line >= 0) && l1->car)
        l1->data[line * l1->geo->line + (addr & (l1->geo->line - 1))] = val;
    else
        l1->store(l1->priv, addr, val);
}

/* Lines from memory (to_l1), or back to it: all, or only the Modified ones. */
static void
cache_l1_lines(cache_l1_t *l1, int to_l1, int modified_only)
{
    int size = l1->geo->line;

    for (int line = 0; line < (l1->geo->sets * l1->geo->ways); line++) {
        if (!l1->tag[line] || (modified_only && !(l1->state[line] & LINE_MODIFIED)))
            continue;
        for (int i = 0; i < size; i++) {
            uint32_t addr = (l1->tag[line] & ~1u) + i;
            if (to_l1)
                l1->data[line * size + i] = l1->load(l1->priv, addr);
            else
                l1->store(l1->priv, addr, l1->data[line * size + i]);
        }
    }
}

int
cache_l1_active(const cache_l1_t *l1)
{
    return l1->track || l1->car;
}

/* CPU_CACHE_* from cpu_cache_hook */
void
cache_l1_event(cache_l1_t *l1, int event)
{
    int car = (cr0 & (CR0_CD | CR0_NW)) == (CR0_CD | CR0_NW);

    if (event != CPU_CACHE_CR0) {
        if (l1->car && (event == CPU_CACHE_WBINVD))
            cache_l1_lines(l1, 0, 1);
        cache_l1_clear(l1);
        l1->car   = 0;
        l1->track = event == CPU_CACHE_INVD;
        if (l1->track)
            timer_on_auto(&l1->timer, 10000.0);
    } else if (l1->track && (cr0 & CR0_CD)) {
        l1->track = 0;
        l1->car   = car;
        if (car)
            cache_l1_lines(l1, 1, 0);
        else
            cache_l1_clear(l1);
    } else if (l1->car && !car) {
        cache_l1_lines(l1, 0, 0);
        cache_l1_clear(l1);
        l1->car = 0;
    }
    l1->update(l1->priv);
}

/* Recording stops when no CD/NW follows soon after INVD: the cache is just on. */
static void
cache_l1_timeout(void *priv)
{
    cache_l1_t *l1 = (cache_l1_t *) priv;

    if (l1->track) {
        l1->track = 0;
        cache_l1_clear(l1);
        l1->update(l1->priv);
    }
}

void
cache_l1_reset(cache_l1_t *l1)
{
    cache_l1_clear(l1);
    l1->track = l1->car = 0;
    timer_disable(&l1->timer);
}

void
cache_l1_init(cache_l1_t *l1)
{
    timer_add(&l1->timer, cache_l1_timeout, l1, 0);
    cache_l1_reset(l1);
}

void
cache_l1_close(cache_l1_t *l1)
{
    timer_disable(&l1->timer);
}

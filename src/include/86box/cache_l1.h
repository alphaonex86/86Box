/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the CPU's L1 cache as RAM (486 and P5 CR0.CD/NW).
 */
#ifndef EMU_CACHE_L1_H
#define EMU_CACHE_L1_H

typedef struct cache_l1_geometry_t {
    int sets;
    int ways;
    int line; /* bytes */
    int lru;  /* replacement: least recently used of 2 ways, else round robin */
} cache_l1_geometry_t;

extern const cache_l1_geometry_t cache_l1_486;  /* 16 KB, 4-way, 16-byte lines (Am5x86) */
extern const cache_l1_geometry_t cache_l1_p54c; /* data cache: 8 KB, 2-way, 32-byte lines */

/* A line fill as the chipset answers it: KEN# and WB/WT# */
enum { CACHE_L1_UNCACHED, CACHE_L1_WT, CACHE_L1_WB };

typedef struct cache_l1_t {
    const cache_l1_geometry_t *geo;
    void                      *priv;
    uint8_t                  (*load)(void *priv, uint32_t addr); /* the memory behind */
    void                     (*store)(void *priv, uint32_t addr, uint8_t val);
    int                      (*fill)(void *priv, uint32_t addr); /* NULL: every line, write-through */
    void                     (*update)(void *priv);              /* cache_l1_active() may have changed */

    uint32_t   tag[1024]; /* line address | 1, 0: invalid */
    uint8_t    state[1024];
    uint8_t    data[16384];
    uint8_t    next[256]; /* per set: the way to replace */
    int        track;
    int        car;
    pc_timer_t timer;
} cache_l1_t;

extern void    cache_l1_init(cache_l1_t *l1); /* geo, priv and the callbacks set */
extern void    cache_l1_reset(cache_l1_t *l1);
extern void    cache_l1_close(cache_l1_t *l1);
extern void    cache_l1_event(cache_l1_t *l1, int event);
extern int     cache_l1_active(const cache_l1_t *l1);
extern uint8_t cache_l1_read(cache_l1_t *l1, uint32_t addr);
extern void    cache_l1_write(cache_l1_t *l1, uint32_t addr, uint8_t val);

#endif /*EMU_CACHE_L1_H*/

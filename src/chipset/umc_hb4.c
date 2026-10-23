/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the UMC HB4 "Super Energy Star Green" PCI Chipset.
 *
 * Note:    This chipset has no datasheet, everything were done via
 *          reverse engineering the BIOS of various machines using it.
 *
 * Note 2:  Additional information were also used from all
 *          around the web.
 *
 * Authors: Tiseno100,
 *          Miran Grca, <mgrca8@gmail.com>
 *
 *          Copyright 2021 Tiseno100.
 *          Copyright 2021-2024 Miran Grca.
 */

/*
   UMC HB4 Configuration Registers

   Sources & Notes:
   Cache registers were found at Vogons: https://www.vogons.org/viewtopic.php?f=46&t=68829&start=20
   Basic Reverse engineering effort was done personally by me

   Warning: Register documentation may be inaccurate!

   UMC 8881x:

   Register 50:
   Bit 7: Enable L2 Cache
   Bit 6: Cache Policy (0: Write Thru / 1: Write Back)

   Bit 5-4 Cache Speed
       0 0 Read 3-2-2-2 Write 3T
       0 1 Read 3-1-1-1 Write 3T
       1 0 Read 2-2-2-2 Write 2T
       1 1 Read 2-1-1-1 Write 2T

   Bit 3 Cache Banks (0: 1 Bank / 1: 2 Banks)

   Bit 2-1-0 Cache Size
       0 0 0 0KB
       0 0 1 64KB
       x-x-x Multiplications of 2(64*2 for 0 1 0) till 2MB

   Register 51:
   Bit 7-6 DRAM Read Speed
       5-4 DRAM Write Speed
       0 0 1 Waits
       0 1 1 Waits
       1 0 1 Wait
       1 1 0 Waits

   Bit 3 Resource Lock Enable
   Bit 2 Graphics Adapter (0: VL Bus / 1: PCI Bus)
   Bit 1 L1 WB Policy (0: WT / 1: WB)
   Bit 0 L2 Cache Tag Lenght (0: 7 Bits / 1: 8 Bits)

   Register 52:
   Bit 7: Host-to-PCI Post Write (0: 1 Wait State / 1: 0 Wait States)

   Register 54:
   Bit 7: DC000-DFFFF Read Enable
   Bit 6: D8000-DBFFF Read Enable
   Bit 5: D4000-D7FFF Read Enable
   Bit 4: D0000-D3FFF Read Enable
   Bit 3: CC000-CFFFF Read Enable
   Bit 2: C8000-CBFFF Read Enable
   Bit 1: C0000-C7FFF Read Enable
   Bit 0: E0000-EFFFF Read Enable

   Register 55:
   Bit 7: F0000-FFFF Read Enable
   Bit 6: Shadow Write Status (1: Write Protect/0: Write)

   Register 56h & 57h: DRAM Bank 0 Configuration
   Register 58h & 59h: DRAM Bank 1 Configuration

   Register 5A:
   Bit 2: Detrubo

   Register 5C:
   Bits 7-0: SMRAM base A27-A20

   Register 5D:
   Bits 3-0: SMRAM base A31-A28

   Register 60:
   Bit 5: If set and SMRAM is enabled, data cycles go to PCI and code cycles go to DRAM
   Bit 0: SMRAM Local Access Enable - if set, SMRAM is also enabled outside SMM
          SMRAM appears to always be enabled in SMM, and always set to A0000-BFFFF.
*/

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include "x86.h"
#include <86box/timer.h>
#include <86box/io.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/dma.h>
#include <86box/pci.h>
#include <86box/plat_unused.h>
#include <86box/port_92.h>
#include <86box/smram.h>
#include <86box/chipset.h>
#include <86box/cache_l1.h>

#ifdef ENABLE_HB4_LOG
int hb4_do_log = ENABLE_HB4_LOG;

static void
hb4_log(const char *fmt, ...)
{
    va_list ap;

    if (hb4_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define hb4_log(fmt, ...)
#endif

typedef struct hb4_simm_t {
    uint8_t  cols;
    uint8_t  rows;
    uint8_t  sides;
    uint32_t offset; /* in simm_data */
} hb4_simm_t;

/* Device option value -> SIMM: column and row address bits, sides */
static const hb4_simm_t simm_types[] = {
    { 0 }, { 10, 10, 1 }, { 10, 10, 2 }, { 11, 11, 1 }, { 10, 12, 1 },
    { 11, 11, 2 }, { 10, 12, 2 }, { 11, 12, 2 }, { 12, 12, 1 }, { 12, 12, 2 }
};

typedef struct hb4_t {
    uint8_t idx;
    uint8_t access_data;

    uint8_t  pci_slot;

    uint8_t  pci_conf[256]; /* PCI Registers */

    int      mem_state[9];

    uint32_t smram_base;

    smram_t *smram;         /* SMRAM Handler */

    /* DRAM/L2 model, see hb4_dram_decode() */
    int           model;
    hb4_simm_t    simm[4];
    uint8_t      *simm_data;
    uint32_t      l2_size; /* 0: none installed */
    int           l2_banks;
    uint8_t      *l2_data;
    uint8_t      *l2_tag;     /* per 16-byte line: tag, bit 7 = dirty with 7-bit tags (51h bit 0 clear) */
    uint8_t      *l2_bad;     /* line not equal to DRAM: never filled, or DRAM/SRAM changed behind it */
    uint32_t      l2_fault[3]; /* BOX86_L2_FAULT=bank:lane:mask: those data bits of that SRAM chip read 0 */
    uint8_t       l2_tag_fault; /* BOX86_L2_TAG_FAULT=mask: those bits of the tag RAM chip read 0 */
    int           l2_touched; /* 50h/5Ah written since the last settle check */
    int           l2_settled; /* no bad line and L2 on: transparent for the CPU, the fast RAM path is back */
    pc_timer_t    l2_timer;
    uint8_t       regs[3];      /* 52h, 53h, 61h */
    int           flat;         /* one-to-one decode: data in 86Box RAM below flat_top, laid out by flat_regs */
    int           overlay;      /* hb4_ram_* handlers in front of the RAM (cache-as-RAM, forced L2 hit) */
    uint32_t      flat_top;
    uint8_t       flat_regs[3];
    mem_mapping_t low_mapping;
    mem_mapping_t high_mapping;
    uint32_t      stuck_addr; /* fault injection: this byte reads bit stuck_mask as 0; 0: none */
    uint8_t       stuck_mask;
    mem_mapping_t stuck_mapping;
    cache_l1_t    l1;           /* the CPU's internal cache, for cache-as-RAM */
} hb4_t;

static int shadow_bios[4]  = { (MEM_READ_EXTANY | MEM_WRITE_INTERNAL), (MEM_READ_EXTANY | MEM_WRITE_EXTANY),
                               (MEM_READ_INTERNAL | MEM_WRITE_INTERNAL), (MEM_READ_INTERNAL | MEM_WRITE_EXTANY) };
static int shadow_read[2]  = { MEM_READ_EXTANY, MEM_READ_INTERNAL };
static int shadow_write[2] = { MEM_WRITE_INTERNAL, MEM_WRITE_EXTANY };

int
hb4_shadow_bios_high(hb4_t *dev)
{
    int state;

    state = shadow_bios[dev->pci_conf[0x55] >> 6];

    if (state != dev->mem_state[8]) {
        mem_set_mem_state_both(0xf0000, 0x10000, state);
        if ((dev->mem_state[8] & MEM_READ_INTERNAL) && !(state & MEM_READ_INTERNAL))
            mem_invalidate_range(0xf0000, 0xfffff);
        dev->mem_state[8] = state;
        return 1;
    }

    return 0;
}

int
hb4_shadow_bios_low(hb4_t *dev)
{
    int state;

    /* Erratum in Vogons' datasheet: Register 55h bit 7 in fact controls E0000-FFFFF. */
    state  = (dev->pci_conf[0x55] & 0x80) ? shadow_read[dev->pci_conf[0x54] & 0x01] :
                                            MEM_READ_EXTANY;
    state |= shadow_write[(dev->pci_conf[0x55] >> 6) & 0x01];

    if (state != dev->mem_state[7]) {
        mem_set_mem_state_both(0xe0000, 0x10000, state);
        dev->mem_state[7] = state;
        return 1;
    }

    return 0;
}

int
hb4_shadow_main(hb4_t *dev)
{
    int state;
    int n = 0;

    for (uint8_t i = 0; i < 6; i++) {
        state  = (dev->pci_conf[0x55] & 0x80) ? shadow_read[(dev->pci_conf[0x54] >> (i + 2)) & 0x01] :
                                                MEM_READ_EXTANY;
        state |= shadow_write[(dev->pci_conf[0x55] >> 6) & 0x01];

        if (state != dev->mem_state[i + 1]) {
            n++;
            mem_set_mem_state_both(0xc8000 + (i << 14), 0x4000, state);
            dev->mem_state[i + 1] = state;
        }
    }

    return n;
}

int
hb4_shadow_video(hb4_t *dev)
{
    int state;

    state  = (dev->pci_conf[0x55] & 0x80) ? shadow_read[(dev->pci_conf[0x54] >> 1) & 0x01] :
                                            MEM_READ_EXTANY;
    state |= shadow_write[(dev->pci_conf[0x55] >> 6) & 0x01];

    if (state != dev->mem_state[0]) {
        mem_set_mem_state_both(0xc0000, 0x8000, state);
        dev->mem_state[0] = state;
        return 1;
    }

    return 0;
}

void
hb4_shadow(hb4_t *dev)
{
    int n = 0;
    hb4_log("SHADOW: %02X%02X\n", dev->pci_conf[0x55], dev->pci_conf[0x54]);

    n = hb4_shadow_bios_high(dev);
    n += hb4_shadow_bios_low(dev);
    n += hb4_shadow_main(dev);
    n += hb4_shadow_video(dev);

    if (n > 0)
        flushmmucache_nopc();
}

static void
hb4_smram(hb4_t *dev)
{
    smram_disable_all();

    dev->smram_base = ((uint32_t) dev->pci_conf[0x5c]) << 20;
    dev->smram_base |= ((uint32_t) (dev->pci_conf[0x5d] & 0x0f)) << 28;
    dev->smram_base |= 0x000a0000;

    /* Bit 0, if set, enables SMRAM access outside SMM. SMRAM appears to be always enabled
       in SMM, and is always set to A0000-BFFFF. */
    smram_enable(dev->smram, dev->smram_base, 0x000a0000, 0x20000, dev->pci_conf[0x60] & 0x01, 1);

    /* Bit 5 seems to set data to go to PCI and code to DRAM. The Samsung SPC7700P-LW uses
       this. */
    if (dev->pci_conf[0x60] & 0x20) {
        if (dev->pci_conf[0x60] & 0x01)
            mem_set_mem_state_smram_ex(0, dev->smram_base, 0x20000, 0x02);
        mem_set_mem_state_smram_ex(1, dev->smram_base, 0x20000, 0x02);
    }

    flushmmucache();
}

/*
   DRAM model (fork), enabled when a SIMM is set in the device options: CPU addresses are decoded to
   SIMM cells as registers 52h/53h/61h program them, so a BIOS sizing the banks sees the aliases and
   holes of real hardware (UM8881F documentation in the repository, coreboot_486 raminit):
   52h bits 0-2/4-6: region L (at 0) and U (above L) of 1 << (20 + n) bytes, bit 3: banks 2-3 form L;
   53h bits 0/2: two banks, 1/3: double sided, 4/5: upper bank of the pair; 61h bits 4/5: MA11.
   Within a side, address bits 2-10 drive column MA0-8 and bits 11-20 row MA0-9; ma_high[] gives the
   rest. While that decode is one-to-one the data is in flat 86Box RAM, else in simm_data. The hb4_ram_*
   handlers serve 0-640K and 1M-256M when the decode is not one-to-one or the internal cache (as RAM)
   or the L2 SRAM (forced hit) answers; code can only run from RAM while it is flat.
*/
#define MA_ROW 0x10
#define MA_NC  0xff

/* Address bits 21-25 -> MA line (column n or MA_ROW | row n), without and with 61h MA11 */
static const uint8_t ma_high[2][5] = {
    { 9, MA_ROW | 10, 10, MA_NC, MA_NC },
    { MA_ROW | 10, MA_ROW | 11, 9, 10, 11 }
};

typedef struct hb4_region_t {
    uint32_t       base;
    uint32_t       size;    /* 0: disabled */
    int            banks;   /* 1 or 2 */
    int            sides;   /* 1 or 2 */
    int            bits;    /* address bits of one side */
    int            first;   /* SIMM of the region's first bank */
    int            swap;    /* one bank: the upper of the pair */
    const uint8_t *ma;
} hb4_region_t;

static hb4_region_t
hb4_region(const uint8_t *regs, int upper)
{
    hb4_region_t r;
    int          code_l = regs[0] & 7;
    int          code   = upper ? ((regs[0] >> 4) & 7) : code_l;
    int          cfg    = regs[1] >> (upper ? 2 : 0);

    r.base  = (upper && code_l) ? (1u << (20 + code_l)) : 0;
    r.size  = (code && code_l) ? (1u << (20 + code)) : 0; /* no region L: no RAM at all */
    r.banks = (cfg & 1) ? 2 : 1;
    r.sides = (cfg & 2) ? 2 : 1;
    r.bits  = 20 + code - r.banks - r.sides + 2;
    r.first = (upper ^ !!(regs[0] & 8)) * 2;
    r.swap  = (regs[1] >> (upper ? 5 : 4)) & 1;
    r.ma    = ma_high[(regs[2] >> (upper ? 5 : 4)) & 1];
    return r;
}

static int
hb4_ma_present(const hb4_simm_t *simm, uint8_t ma)
{
    if (ma == MA_NC)
        return 0;
    return (ma & MA_ROW) ? ((ma & 0x0f) < simm->rows) : (ma < simm->cols);
}

static int64_t
hb4_dram_decode(const hb4_t *dev, const uint8_t *regs, uint32_t addr)
{
    hb4_region_t r = hb4_region(regs, 0);

    if (addr - r.base >= r.size)
        r = hb4_region(regs, 1);
    if (addr - r.base >= r.size)
        return -1;

    uint32_t          offset = addr - r.base;
    uint32_t          above  = offset >> r.bits; /* bank of the pair, then side */
    int               bank   = ((r.banks == 2) ? (above >> (r.sides - 1)) : 0) ^ r.swap;
    uint32_t          side   = above & (r.sides - 1);
    const hb4_simm_t *simm   = &dev->simm[r.first + bank];
    uint32_t          col    = (offset >> 2) & 0x1ff;
    uint32_t          row    = (offset >> 11) & 0x3ff;

    if (side >= simm->sides)
        return -1;
    for (int bit = 21; bit < MIN(r.bits, 26); bit++) {
        uint8_t ma = r.ma[bit - 21];
        if (((offset >> bit) & 1) && (ma != MA_NC)) {
            if (ma & MA_ROW)
                row |= 1 << (ma & 0x0f);
            else
                col |= 1 << ma;
        }
    }
    col &= (1u << simm->cols) - 1;
    row &= (1u << simm->rows) - 1;
    return simm->offset + ((uint64_t) side << (simm->cols + simm->rows + 2)) +
           ((((row << simm->cols) | col) << 2) | (addr & 3));
}

/* One-to-one decode: every bank used has the sides and every side address bit drives an MA line the
   SIMM has. */
static int
hb4_region_flat(const hb4_t *dev, const uint8_t *regs, int upper)
{
    hb4_region_t r = hb4_region(regs, upper);

    for (int bank = 0; r.size && (bank < 2); bank++) {
        const hb4_simm_t *simm = &dev->simm[r.first + bank];
        if ((r.banks == 1) && (bank != r.swap))
            continue;
        if (simm->sides < r.sides)
            return 0;
        for (int bit = 21; bit < r.bits; bit++)
            if ((bit >= 26) || !hb4_ma_present(simm, r.ma[bit - 21]))
                return 0;
    }
    return 1;
}

static int
hb4_l2_forced(const hb4_t *dev)
{
    return dev->pci_conf[0x5a] & 0x40; /* even with the L2 disabled */
}

/* L2 data SRAM in forced hit mode: index wraps at the installed and at the programmed size; with two
   banks A2 selects the bank, so a one-bank setting sees each cell twice. */
static uint8_t *
hb4_l2_byte(hb4_t *dev, uint32_t addr)
{
    uint8_t  r50  = dev->pci_conf[0x50];
    uint32_t size = MIN(dev->l2_size, 0x8000u << (r50 & 7));

    if (r50 & 0x08) {
        if ((dev->l2_banks == 1) && (addr & 4))
            return NULL;
    } else if (dev->l2_banks == 2)
        addr &= ~4;
    return size ? &dev->l2_data[addr & (size - 1)] : NULL;
}

/* A byte read from the L2 data SRAM, chip by bank (A2 with two banks) and byte lane, with the injected fault */
static uint8_t
hb4_l2_sram(const hb4_t *dev, uint32_t addr, uint8_t val)
{
    if (dev->l2_fault[2] && ((addr >> 2 & 1) == dev->l2_fault[0]) && ((addr & 3) == dev->l2_fault[1]))
        val &= ~dev->l2_fault[2];
    return val;
}

/* DRAM behind the caches: flat RAM or SIMM cells; unanswered reads float high. */
static uint8_t
hb4_dram_load(hb4_t *dev, uint32_t addr)
{
    if (dev->flat)
        return (addr < dev->flat_top) ? ram[addr] : 0xff;
    int64_t offset = hb4_dram_decode(dev, dev->regs, addr);
    return (offset < 0) ? 0xff : dev->simm_data[offset];
}

static void
hb4_dram_store(hb4_t *dev, uint32_t addr, uint8_t val)
{
    if (dev->flat) {
        if (addr < dev->flat_top)
            mem_write_ramb_page(addr, val, &pages[addr >> 12]); /* tracks code pages */
        return;
    }
    int64_t offset = hb4_dram_decode(dev, dev->regs, addr);
    if (offset >= 0)
        dev->simm_data[offset] = val;
}

/*
   L2 normal mode (fork), modelled while some line differs from DRAM (power-on contents, DRAM written with
   the L2 off, forced-hit writes); with none and the L2 on it is transparent and the fast RAM path returns. Direct mapped, 16-byte lines, no valid bit; tags, dirty bits and data
   are random at power-on. Hypothesis fitted to the real board (an L2 with its size set and bit 7 clear
   crashed it once write-back was set; the original BIOS fills twice, off then on): with a size programmed
   the L2 fills lines on read misses even while 50h bit 7 is clear, but serves hits only with bit 7 set;
   in write-back (bit 6) a miss first writes a dirty line back, in write-through a fill keeps the dirty bit.
*/
static uint32_t
hb4_l2_prog(const hb4_t *dev)
{
    uint8_t r50 = dev->pci_conf[0x50];
    return (dev->l2_size && (r50 & 7)) ? MIN(dev->l2_size, 0x8000u << (r50 & 7)) : 0;
}

static int
hb4_l2_active(const hb4_t *dev)
{
    return !dev->l2_settled && hb4_l2_prog(dev) && !hb4_l2_forced(dev);
}

static uint8_t *
hb4_l2_slot(hb4_t *dev, uint32_t addr, uint32_t *line)
{
    uint32_t prog = hb4_l2_prog(dev), mask = (dev->pci_conf[0x51] & 1) ? 0xff : 0x7f;
    uint32_t top  = dev->flat ? dev->flat_top : 0x10000000u;
    if (!prog || (addr >= MIN((uint64_t) prog * (mask + 1), top)))
        return NULL;
    uint8_t *byte = hb4_l2_byte(dev, addr & ~15u);
    if (!byte)
        return NULL;
    *line = (byte - dev->l2_data) >> 4;
    return byte;
}

static void hb4_ram_update(hb4_t *dev);

/* Back to modelling after a transparent period: the L2 then held what DRAM holds, line by line. */
static void
hb4_l2_resume(hb4_t *dev)
{
    uint32_t prog = hb4_l2_prog(dev);
    uint8_t  mask = (dev->pci_conf[0x51] & 1) ? 0xff : 0x7f;

    for (uint32_t line = 0; prog && (line < prog / 16); line++) {
        for (int i = 0; i < 16; i++)
            dev->l2_data[line * 16 + i] = hb4_dram_load(dev, (dev->l2_tag[line] & mask) * prog + line * 16 + i);
        dev->l2_tag[line] &= mask;
    }
    dev->l2_settled = 0;
}

/* Every 0.5 s while modelled: once every line equals DRAM with the L2 on and its registers untouched
   since the last check (POST done), the L2 is transparent and is left to the fast RAM path (DRAM is
   kept current, see hb4_l2_store). Not with CR0.CD set (L1 off: a firmware L2 init in progress, which
   reads lines, then changes DRAM behind the L2: the reads must still load tags). */
static void
hb4_l2_settle(void *priv)
{
    hb4_t   *dev  = (hb4_t *) priv;
    uint32_t prog = hb4_l2_prog(dev);
    int      bad  = !(dev->pci_conf[0x50] & 0x80) || !prog || hb4_l2_forced(dev) || (cr0 & (1u << 30));

    for (uint32_t line = 0; !bad && (line < prog / 16); line++)
        bad = dev->l2_bad[line];
    if (bad || dev->l2_touched) {
        dev->l2_touched = 0;
        if (hb4_l2_active(dev))
            timer_on_auto(&dev->l2_timer, 500000.0);
        return;
    }
    dev->l2_settled = 1;
    hb4_ram_update(dev);
}

static uint8_t
hb4_l2_load(hb4_t *dev, uint32_t addr)
{
    uint32_t line, prog = hb4_l2_prog(dev);
    uint8_t *data = hb4_l2_slot(dev, addr, &line), r50 = dev->pci_conf[0x50];
    uint8_t  mask = (dev->pci_conf[0x51] & 1) ? 0xff : 0x7f, tag = (addr / prog) & mask;
    uint32_t base = addr & (prog - 1) & ~15u;

    if (!data)
        return hb4_dram_load(dev, addr);
    if ((r50 & 0x80) && ((dev->l2_tag[line] & mask) == tag))
        return hb4_l2_sram(dev, addr, data[addr & 15]);
    if ((r50 & 0x40) && ((mask == 0xff) || (dev->l2_tag[line] & 0x80)))
        for (int i = 0; i < 16; i++)
            hb4_dram_store(dev, (dev->l2_tag[line] & mask) * prog + base + i, data[i]);
    for (int i = 0; i < 16; i++)
        data[i] = hb4_dram_load(dev, (addr & ~15u) + i);
    dev->l2_tag[line] = (tag | ((r50 & 0x40) ? 0 : (dev->l2_tag[line] & 0x80 & ~mask))) & ~dev->l2_tag_fault;
    dev->l2_bad[line] = 0;
    return data[addr & 15];
}

static void
hb4_l2_store(hb4_t *dev, uint32_t addr, uint8_t val)
{
    uint32_t line, prog = hb4_l2_prog(dev);
    uint8_t *data = hb4_l2_slot(dev, addr, &line), r50 = dev->pci_conf[0x50];
    uint8_t  mask = (dev->pci_conf[0x51] & 1) ? 0xff : 0x7f;

    int hit = data && ((dev->l2_tag[line] & mask) == ((addr / prog) & mask));
    if (hit && !(r50 & 0x80))
        dev->l2_bad[line] = 1;        /* DRAM changes behind the line */
    if (hit && (r50 & 0x80)) {
        data[addr & 15] = val;
        if ((r50 & 0x40) && (mask != 0xff))
            dev->l2_tag[line] = (dev->l2_tag[line] | 0x80) & ~dev->l2_tag_fault;
    }
    /* also in write-back: code is fetched from DRAM directly (exec pointer), and a later write-back of
       this line writes the same bytes; random power-on dirty lines still write garbage back */
    hb4_dram_store(dev, addr, val);
}

/* Behind the internal cache: L2 SRAM (forced hit), the L2 model or DRAM. */
static uint8_t
hb4_load(hb4_t *dev, uint32_t addr)
{
    if (hb4_l2_forced(dev)) {
        uint8_t *byte = hb4_l2_byte(dev, addr);
        return byte ? hb4_l2_sram(dev, addr, *byte) : 0xff;
    }
    return hb4_l2_active(dev) ? hb4_l2_load(dev, addr) : hb4_dram_load(dev, addr);
}

static void
hb4_store(hb4_t *dev, uint32_t addr, uint8_t val)
{
    if (hb4_l2_forced(dev)) {
        uint8_t *byte = hb4_l2_byte(dev, addr);
        if (byte) {
            *byte = val;
            dev->l2_bad[(byte - dev->l2_data) >> 4] = 1;
        }
    } else if (hb4_l2_active(dev))
        hb4_l2_store(dev, addr, val);
    else
        hb4_dram_store(dev, addr, val);
}

static uint8_t
hb4_l1_load(void *priv, uint32_t addr)
{
    return hb4_load((hb4_t *) priv, addr);
}

static void
hb4_l1_store(void *priv, uint32_t addr, uint8_t val)
{
    hb4_store((hb4_t *) priv, addr, val);
}

static uint8_t
hb4_ram_readb(uint32_t addr, void *priv)
{
    return cache_l1_read(&((hb4_t *) priv)->l1, addr);
}

static uint16_t
hb4_ram_readw(uint32_t addr, void *priv)
{
    return hb4_ram_readb(addr, priv) | (hb4_ram_readb(addr + 1, priv) << 8);
}

static uint32_t
hb4_ram_readl(uint32_t addr, void *priv)
{
    return hb4_ram_readw(addr, priv) | ((uint32_t) hb4_ram_readw(addr + 2, priv) << 16);
}

static void
hb4_ram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    cache_l1_write(&((hb4_t *) priv)->l1, addr, val);
}

static void
hb4_ram_writew(uint32_t addr, uint16_t val, void *priv)
{
    hb4_ram_writeb(addr, val, priv);
    hb4_ram_writeb(addr + 1, val >> 8, priv);
}

static void
hb4_ram_writel(uint32_t addr, uint32_t val, void *priv)
{
    hb4_ram_writew(addr, val, priv);
    hb4_ram_writew(addr + 2, val >> 16, priv);
}

static uint8_t
hb4_stuck_readb(uint32_t addr, void *priv)
{
    hb4_t  *dev = (hb4_t *) priv;
    uint8_t val = hb4_ram_readb(addr, priv);

    return (addr == dev->stuck_addr) ? (val & ~dev->stuck_mask) : val;
}

static uint16_t
hb4_stuck_readw(uint32_t addr, void *priv)
{
    return hb4_stuck_readb(addr, priv) | (hb4_stuck_readb(addr + 1, priv) << 8);
}

static uint32_t
hb4_stuck_readl(uint32_t addr, void *priv)
{
    return hb4_stuck_readw(addr, priv) | ((uint32_t) hb4_stuck_readw(addr + 2, priv) << 16);
}

/* Copies flat 86Box RAM from (to_ram: to) the SIMMs; 2 KB runs are contiguous in both. */
static void
hb4_ram_copy(hb4_t *dev, const uint8_t *regs, uint32_t top, int to_ram)
{
    for (uint32_t addr = 0; addr < top; addr += 0x800) {
        if (addr == 0xa0000)
            addr = 0x100000;

        int64_t offset = hb4_dram_decode(dev, regs, addr);
        if (offset < 0)
            fatal("UMC HB4: flat RAM at %08X not decoded\n", addr);
        if (to_ram)
            memcpy(ram + addr, dev->simm_data + offset, 0x800);
        else
            memcpy(dev->simm_data + offset, ram + addr, 0x800);
    }
    if (to_ram)
        mem_invalidate_range(0, top - 1);
}

static void
hb4_ram_update(hb4_t *dev)
{
    const uint8_t *regs = dev->regs;

    dev->regs[0]     = dev->pci_conf[0x52];
    dev->regs[1]     = dev->pci_conf[0x53];
    dev->regs[2]     = dev->pci_conf[0x61];
    uint32_t top     = hb4_region(regs, 0).size + hb4_region(regs, 1).size;
    int      flat    = (top >= 0x100000) && (top <= mem_size * 1024) && hb4_region_flat(dev, regs, 0) &&
                       hb4_region_flat(dev, regs, 1);
    int      overlay = cache_l1_active(&dev->l1) || hb4_l2_forced(dev) || hb4_l2_active(dev);

    if ((flat != dev->flat) || (flat && memcmp(regs, dev->flat_regs, 3))) {
        if (dev->flat)
            hb4_ram_copy(dev, dev->flat_regs, dev->flat_top, 0);
        dev->flat = flat;
        if (flat) {
            memcpy(dev->flat_regs, regs, 3);
            dev->flat_top = top;
            hb4_ram_copy(dev, regs, top, 1);
            /* PCI memory may sit between the RAM top and the 86Box RAM size */
            mem_set_mem_state_both(0x100000, top - 0x100000, MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
            mem_set_mem_state_both(top, mem_size * 1024 - top, MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);
        }
    } else if (overlay == dev->overlay)
        return;

    dev->overlay = overlay;
    if (flat && !overlay) {
        mem_mapping_disable(&dev->low_mapping);
        mem_mapping_disable(&dev->high_mapping);
        mem_mapping_set_addr(&ram_high_mapping, 0x100000, top - 0x100000);
    } else {
        mem_mapping_set_exec(&dev->low_mapping, flat ? ram : NULL);   /* code fetches bypass the L2 model */
        mem_mapping_enable(&dev->low_mapping);
        mem_mapping_set_addr(&dev->high_mapping, 0x100000, (flat ? top : 0x10000000) - 0x100000);
        mem_mapping_set_exec(&dev->high_mapping, flat ? (ram + 0x100000) : NULL);
    }
    if (dev->stuck_addr)
        mem_mapping_set_exec(&dev->stuck_mapping, flat ? (ram + (dev->stuck_addr & ~0xfff)) : NULL);
    flushmmucache();
}

static hb4_t *hb4_model;

static void
hb4_cache_event(int event)
{
    hb4_t *dev = hb4_model;

    /* on the real chip a flush also empties the L2: logged so tests can count them (slow POST) */
    if ((event == CPU_CACHE_WBINVD) && (dev->pci_conf[0x50] & 0x80) && hb4_l2_prog(dev))
        fprintf(stderr, "UMC HB4: WBINVD with the L2 on\n");
    cache_l1_event(&dev->l1, event);
}

static void
hb4_l1_update(void *priv)
{
    hb4_ram_update((hb4_t *) priv);
}

static void
hb4_write(UNUSED(int func), int addr, UNUSED(int len), uint8_t val, void *priv)
{
    hb4_t *dev = (hb4_t *) priv;

    hb4_log("UM8881: dev->regs[%02x] = %02x POST: %02x \n", addr, val, inb(0x80));

    switch (addr) {
        case 0x04:
        case 0x05:
            dev->pci_conf[addr] = val;
            break;

        case 0x07:
            dev->pci_conf[addr] &= ~(val & 0xf9);
            break;

        case 0x0c:
        case 0x0d:
            dev->pci_conf[addr] = val;
            break;

        case 0x50:
            if (dev->model && dev->l2_settled && (((val & 0x87) != (dev->pci_conf[0x50] & 0x87)) || !(val & 0x80)))
                hb4_l2_resume(dev);
            dev->l2_touched = 1;
            if (dev->model && !dev->l2_settled)
                timer_on_auto(&dev->l2_timer, 500000.0);
            dev->pci_conf[addr]   = dev->model ? val : ((val & 0xf8) | 4); /* Hardcode Cache Size to 512KB */
            cpu_cache_ext_enabled = !!(val & 0x80);     /* Fixes freezing issues on the HOT-433A*/
            cpu_update_waitstates();
            if (dev->model)
                hb4_ram_update(dev);
            break;

        case 0x51:
            dev->pci_conf[addr] = val;
            break;

        case 0x52 ... 0x53:
            dev->pci_conf[addr] = val;
            if (dev->model)
                hb4_ram_update(dev);
            break;

        case 0x54 ... 0x55:
            dev->pci_conf[addr] = val;
            hb4_shadow(dev);
            break;

        case 0x56 ... 0x59:
        case 0x5e ... 0x5f:
            dev->pci_conf[addr] = val;
            break;

        case 0x5a:
            if (dev->model && dev->l2_settled && (val & 0x40))
                hb4_l2_resume(dev);
            dev->l2_touched = 1;
            if (dev->model && !dev->l2_settled)
                timer_on_auto(&dev->l2_timer, 500000.0);
            dev->pci_conf[addr] = val;
            if (dev->model)
                hb4_ram_update(dev);
            break;

        case 0x5c ... 0x5d:
        case 0x60:
            dev->pci_conf[addr] = val;
            hb4_smram(dev);
            break;

        case 0x61:
            dev->pci_conf[addr] = val;
            if (dev->model)
                hb4_ram_update(dev);
            break;

        case 0x62:
            dev->pci_conf[addr] = val & 0x03;
            break;

        default:
            break;
    }
}

static uint8_t
hb4_read(int func, int addr, UNUSED(int len), void *priv)
{
    const hb4_t  *dev = (hb4_t *) priv;
    uint8_t       ret = 0xff;

    if (func == 0)
        ret = dev->pci_conf[addr];

    return ret;
}

static void
hb4_reset(void *priv)
{
    hb4_t *dev = (hb4_t *) priv;
    memset(dev->pci_conf, 0x00, sizeof(dev->pci_conf));

    dev->pci_conf[0x00] = 0x60; /* UMC */
    dev->pci_conf[0x01] = 0x10;
    dev->pci_conf[0x02] = 0x81; /* 8881x */
    dev->pci_conf[0x03] = 0x88;
    dev->pci_conf[0x07] = 0x02;
    dev->pci_conf[0x08] = 0x04;
    dev->pci_conf[0x09] = 0x00;
    dev->pci_conf[0x0a] = 0x00;
    dev->pci_conf[0x0b] = 0x06;
    dev->pci_conf[0x50] = 0x00;
    dev->pci_conf[0x51] = 0x00;
    dev->pci_conf[0x52] = 0x01;
    dev->pci_conf[0x53] = 0x00;
    dev->pci_conf[0x54] = 0x00;
    dev->pci_conf[0x55] = 0x40;
    dev->pci_conf[0x56] = 0xff;
    dev->pci_conf[0x57] = 0x0f;
    dev->pci_conf[0x58] = 0xff;
    dev->pci_conf[0x59] = 0x0f;
    dev->pci_conf[0x5a] = 0x00;
    dev->pci_conf[0x5b] = 0x2c;
    dev->pci_conf[0x5c] = 0x00;
    dev->pci_conf[0x5d] = 0x0f;
    dev->pci_conf[0x5e] = 0x00;
    dev->pci_conf[0x5f] = 0xff;
    dev->pci_conf[0x60] = 0x00;
    dev->pci_conf[0x61] = 0x00;
    dev->pci_conf[0x62] = 0x00;

    hb4_shadow(dev);
    hb4_smram(dev);

    cpu_cache_ext_enabled = 0;
    cpu_update_waitstates();

    memset(dev->mem_state, 0x00, sizeof(dev->mem_state));

    if (dev->model) {
        cache_l1_reset(&dev->l1);
        hb4_ram_update(dev);
    }
}

static void
hb4_close(void *priv)
{
    hb4_t *dev = (hb4_t *) priv;

    if (dev->model) {
        cpu_cache_hook = NULL;
        cache_l1_close(&dev->l1);
        mem_mapping_disable(&dev->low_mapping);
        mem_mapping_disable(&dev->high_mapping);
        if (dev->stuck_addr)
            mem_mapping_disable(&dev->stuck_mapping);
        free(dev->simm_data);
        free(dev->l2_data);
    }
    smram_del(dev->smram);
    free(dev);
}

static void *
hb4_init(UNUSED(const device_t *info))
{
    hb4_t *dev = (hb4_t *) calloc(1, sizeof(hb4_t));

    pci_add_card(PCI_ADD_NORTHBRIDGE, hb4_read, hb4_write, dev, &dev->pci_slot); /* Device 10: UMC 8881x */

    /* Port 92: A20 only. On the real MB-8433UUD-A bit 0 does not reset (Ctrl+Alt+Del hung with a BIOS that
       only used it; the original BIOS resets through the keyboard controller). */
    port_92_set_features(device_add(&port_92_pci_device), 0, 1);

    /* SMRAM */
    dev->smram = smram_add();

    dev->smram_base = 0x000a0000;

    uint32_t capacity = 0;
    for (int bank = 0; bank < 4; bank++) {
        char name[8];
        int  type = (snprintf(name, sizeof(name), "bank%d", bank), device_get_config_int(name));
        if ((type < 0) || (type >= (int) (sizeof(simm_types) / sizeof(simm_types[0]))))
            fatal("UMC HB4: bad SIMM type %d in bank %d\n", type, bank);
        dev->simm[bank]        = simm_types[type];
        dev->simm[bank].offset = capacity;
        if (type)
            capacity += dev->simm[bank].sides << (dev->simm[bank].cols + dev->simm[bank].rows + 2);
    }
    dev->model = capacity > 0;
    if (dev->model) {
        dev->l2_size   = device_get_config_int("l2_cache") << 10;
        dev->l2_banks  = device_get_config_int("l2_banks");
        dev->simm_data = calloc(1, capacity);
        dev->l2_data   = malloc(1 << 20);
        dev->l2_tag    = malloc(1 << 16);
        dev->l2_bad    = malloc(1 << 16);
        if (!dev->simm_data || !dev->l2_data || !dev->l2_tag || !dev->l2_bad)
            fatal("UMC HB4: out of memory for %u MB of SIMMs\n", capacity >> 20);
        mem_mapping_add(&dev->low_mapping, 0, 0xa0000, hb4_ram_readb, hb4_ram_readw, hb4_ram_readl,
                        hb4_ram_writeb, hb4_ram_writew, hb4_ram_writel, NULL, 0, dev);
        mem_mapping_add(&dev->high_mapping, 0x100000, 0x0ff00000, hb4_ram_readb, hb4_ram_readw, hb4_ram_readl,
                        hb4_ram_writeb, hb4_ram_writew, hb4_ram_writel, NULL, 0, dev);
        mem_mapping_disable(&dev->low_mapping);
        mem_mapping_disable(&dev->high_mapping);
        dev->stuck_addr = device_get_config_int("stuck_addr");
        dev->stuck_mask = 1 << device_get_config_int("stuck_bit");
        if (dev->stuck_addr) /* added last: in front of the RAM and the mappings above */
            mem_mapping_add(&dev->stuck_mapping, dev->stuck_addr & ~0xfff, 0x1000, hb4_stuck_readb,
                            hb4_stuck_readw, hb4_stuck_readl, hb4_ram_writeb, hb4_ram_writew, hb4_ram_writel,
                            NULL, 0, dev);
        /* power-on SRAM contents: random, reproducible with BOX86_L2_SEED */
        const char *tag_fault = getenv("BOX86_L2_TAG_FAULT");
        if (tag_fault)
            dev->l2_tag_fault = strtoul(tag_fault, NULL, 16);
        const char *fault = getenv("BOX86_L2_FAULT");
        unsigned    bank, lane, bits;
        if (fault && (sscanf(fault, "%u:%u:%x", &bank, &lane, &bits) == 3)) {
            dev->l2_fault[0] = bank & 1;
            dev->l2_fault[1] = lane & 3;
            dev->l2_fault[2] = bits & 0xff;
        }
        const char *seed = getenv("BOX86_L2_SEED");
        uint64_t    x    = seed ? strtoull(seed, NULL, 0) : 1;
        for (uint32_t i = 0; i < (1 << 20); i++) {
            x = x * 6364136223846793005ULL + 1442695040888963407ULL;
            dev->l2_data[i] = x >> 56;
            if (i < (1 << 16))
                dev->l2_tag[i] = x >> 48;
        }
        memset(dev->l2_bad, 1, 1 << 16);
        dev->l1.geo    = &cache_l1_486;
        dev->l1.priv   = dev;
        dev->l1.load   = hb4_l1_load;
        dev->l1.store  = hb4_l1_store;
        dev->l1.update = hb4_l1_update;
        cache_l1_init(&dev->l1);
        timer_add(&dev->l2_timer, hb4_l2_settle, dev, 0);
        hb4_model      = dev;
        cpu_cache_hook = hb4_cache_event;
    }
    hb4_reset(dev);

    return dev;
}

#define HB4_BANK(n)                                                                   \
    {                                                                                 \
        .name = "bank" #n, .description = "SIMM bank " #n, .type = CONFIG_SELECTION,  \
        .default_int = 0, .selection = {                                              \
            { .description = "Empty", .value = 0 },                                  \
            { .description = "4 MB, row/column bits 10/10", .value = 1 },             \
            { .description = "8 MB, 10/10, double sided", .value = 2 },               \
            { .description = "16 MB, 11/11", .value = 3 },                            \
            { .description = "16 MB, 12/10", .value = 4 },                            \
            { .description = "32 MB, 11/11, double sided", .value = 5 },              \
            { .description = "32 MB, 12/10, double sided", .value = 6 },              \
            { .description = "64 MB, 12/11, double sided", .value = 7 },              \
            { .description = "64 MB, 12/12", .value = 8 },                            \
            { .description = "128 MB, 12/12, double sided", .value = 9 },             \
            { .description = "" } }                                                   \
    }

/* All banks empty: no DRAM/L2 model, flat RAM of the machine size. */
static const device_config_t hb4_config[] = {
    HB4_BANK(0), HB4_BANK(1), HB4_BANK(2), HB4_BANK(3),
    {
        .name = "l2_cache", .description = "L2 cache (with SIMMs set)", .type = CONFIG_SELECTION,
        .default_int = 256, .selection = {
            { .description = "None", .value = 0 },
            { .description = "64 KB", .value = 64 },
            { .description = "128 KB", .value = 128 },
            { .description = "256 KB", .value = 256 },
            { .description = "512 KB", .value = 512 },
            { .description = "1 MB", .value = 1024 },
            { .description = "" } }
    },
    {
        .name = "l2_banks", .description = "L2 cache banks", .type = CONFIG_SELECTION,
        .default_int = 1, .selection = {
            { .description = "1", .value = 1 },
            { .description = "2", .value = 2 },
            { .description = "" } }
    },
    {
        .name = "stuck_addr", .description = "Faulty RAM byte (any address in the .cfg)", .type = CONFIG_SELECTION,
        .default_int = 0, .selection = {
            { .description = "None", .value = 0 },
            { .description = "2AA400h", .value = 0x2aa400 },
            { .description = "" } }
    },
    {
        .name = "stuck_bit", .description = "Faulty RAM bit (reads 0)", .type = CONFIG_SELECTION,
        .default_int = 3, .selection = {
            { .description = "0", .value = 0 }, { .description = "1", .value = 1 },
            { .description = "2", .value = 2 }, { .description = "3", .value = 3 },
            { .description = "4", .value = 4 }, { .description = "5", .value = 5 },
            { .description = "6", .value = 6 }, { .description = "7", .value = 7 },
            { .description = "" } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

const device_t umc_hb4_device = {
    .name          = "UMC HB4(8881F)",
    .internal_name = "umc_hb4",
    .flags         = DEVICE_PCI,
    .local         = 0x886a,
    .init          = hb4_init,
    .close         = hb4_close,
    .reset         = hb4_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = hb4_config
};

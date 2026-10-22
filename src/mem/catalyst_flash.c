/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the 28F010 family of 1 Mbit 12 V flash
 *          devices (Intel 28F010, AMD Am28F010, Catalyst CAT28F010).
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *
 *          Copyright 2008-2019 Sarah Walker.
 *          Copyright 2016-2019 Miran Grca.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/machine.h>
#include <86box/timer.h>
#include <86box/nvr.h>
#include <86box/plat.h>
#include <86box/flash.h>

/* The command register of the 28F010 (Intel 290207, AMD Am28F010, Catalyst CAT28F010): a write in a set-up state
   is the operation's data, any other write is a command. Program and erase only start on the set-up's second write;
   the next command (C0h/A0h verify) ends them. Programming clears bits only. Without 12 V on VPP the register is
   disabled: the chip reads its array and ignores every write. */
enum {
    CMD_READ           = 0x00,
    CMD_READ_ID        = 0x90,
    CMD_READ_ID_ALT    = 0x80,
    CMD_ERASE          = 0x20,
    CMD_ERASE_VERIFY   = 0xa0,
    CMD_PROGRAM        = 0x40,
    CMD_PROGRAM_VERIFY = 0xc0,
    CMD_RESET          = 0xff
};

enum {
    STATE_READ,
    STATE_ID,
    STATE_ERASE_SETUP,
    STATE_PROGRAM_SETUP,
    STATE_BUSY
};

typedef struct flash_t {
    uint8_t  state;
    uint8_t  dirty;
    uint16_t id;
    uint8_t *array;

    mem_mapping_t mapping;
    mem_mapping_t mapping_h[2];
} flash_t;

static char flash_path[1024];

/* Board signal VPP: BOX86_FLASH_VPP=5 (jumper at 5 V) leaves the chip read-only. */
static int
flash_vpp_low(void)
{
    const char *v = getenv("BOX86_FLASH_VPP");
    return v && !strcmp(v, "5");
}

static uint8_t
flash_read(uint32_t addr, void *priv)
{
    const flash_t *dev = (flash_t *) priv;

    if (!flash_bios_read_selected(addr))
        return 0xff;

    addr &= biosmask;

    if (dev->state == STATE_ID)
        return (addr & 1) ? (dev->id & 0xff) : (dev->id >> 8);

    return dev->array[addr];
}

static uint16_t
flash_readw(uint32_t addr, void *priv)
{
    return flash_read(addr, priv) | (flash_read(addr + 1, priv) << 8);
}

static uint32_t
flash_readl(uint32_t addr, void *priv)
{
    return flash_readw(addr, priv) | (flash_readw(addr + 2, priv) << 16);
}

static void
flash_write(uint32_t addr, uint8_t val, void *priv)
{
    flash_t *dev = (flash_t *) priv;

    if (!flash_bios_write_selected(addr) || flash_vpp_low())
        return;

    addr &= biosmask;

    switch (dev->state) {
        case STATE_ERASE_SETUP:
            if (val == CMD_ERASE) {
                memset(dev->array, 0xff, biosmask + 1);
                dev->dirty = 1;
                dev->state = STATE_BUSY;
            } else
                dev->state = STATE_READ;
            break;

        case STATE_PROGRAM_SETUP:
            if ((dev->array[addr] & val) != dev->array[addr]) {
                dev->array[addr] &= val;
                dev->dirty = 1;
            }
            dev->state = STATE_BUSY;
            break;

        default:
            switch (val) {
                case CMD_READ:
                case CMD_RESET:
                case CMD_ERASE_VERIFY:
                case CMD_PROGRAM_VERIFY:
                    dev->state = STATE_READ;
                    break;
                case CMD_READ_ID:
                case CMD_READ_ID_ALT:
                    dev->state = STATE_ID;
                    break;
                case CMD_ERASE:
                    dev->state = STATE_ERASE_SETUP;
                    break;
                case CMD_PROGRAM:
                    dev->state = STATE_PROGRAM_SETUP;
                    break;
                default:
                    break;
            }
            break;
    }
}

static void
flash_writew(uint32_t addr, uint16_t val, void *priv)
{
    flash_write(addr, val & 0xff, priv);
    flash_write(addr + 1, val >> 8, priv);
}

static void
flash_writel(uint32_t addr, uint32_t val, void *priv)
{
    flash_writew(addr, val & 0xffff, priv);
    flash_writew(addr + 2, val >> 16, priv);
}

static void
catalyst_flash_add_mappings(flash_t *dev)
{
    memcpy(dev->array, rom, biosmask + 1);

    mem_mapping_add(&dev->mapping, 0xe0000, 0x20000,
                    flash_read, flash_readw, flash_readl,
                    flash_write, flash_writew, flash_writel,
                    dev->array, MEM_MAPPING_EXTERNAL | MEM_MAPPING_ROM | MEM_MAPPING_ROMCS | MEM_MAPPING_ROM_WS, (void *) dev);

    mem_mapping_add(&(dev->mapping_h[0]), 0xfffc0000, 0x20000,
                    flash_read, flash_readw, flash_readl,
                    flash_write, flash_writew, flash_writel,
                    dev->array, MEM_MAPPING_EXTERNAL | MEM_MAPPING_ROM | MEM_MAPPING_ROMCS | MEM_MAPPING_ROM_WS, (void *) dev);
    mem_mapping_add(&(dev->mapping_h[1]), 0xfffe0000, 0x20000,
                    flash_read, flash_readw, flash_readl,
                    flash_write, flash_writew, flash_writel,
                    dev->array, MEM_MAPPING_EXTERNAL | MEM_MAPPING_ROM | MEM_MAPPING_ROMCS | MEM_MAPPING_ROM_WS, (void *) dev);
}

static void
catalyst_flash_reset(void *priv)
{
    flash_t *dev = (flash_t *) priv;

    dev->state = STATE_READ;
}

static void *
catalyst_flash_init(const device_t *info)
{
    FILE    *fp;
    flash_t *dev;

    dev = calloc(1, sizeof(flash_t));

    sprintf(flash_path, "%s.bin", machine_get_nvr_name_ex(machine));

    mem_mapping_disable(&bios_mapping);
    mem_mapping_disable(&bios_high_mapping);

    dev->array = (uint8_t *) calloc(1, 0x20000);
    memset(dev->array, 0xff, 0x20000);

    catalyst_flash_add_mappings(dev);

    dev->state = STATE_READ;
    dev->id    = info->local;

    if (strlen(flash_path) > 0) {
        fp = nvr_fopen(flash_path, "rb");
        if (fp != NULL) {
            if (!dump_missing)
                (void) !fread(dev->array, 0x20000, 1, fp);
            fclose(fp);
        } else if (!dump_missing)
            dev->dirty = 1;
    } else
        fatal("Attempting to open the Flash file for reading with an empty invalid name\n");

    return dev;
}

static void
catalyst_flash_close(void *priv)
{
    FILE    *fp;
    flash_t *dev = (flash_t *) priv;

    if (dev->dirty) {
        if (strlen(flash_path) > 0) {
            fp = nvr_fopen(flash_path, "wb");
            if (fp != NULL) {
                if (!dump_missing)
                    fwrite(dev->array, 0x20000, 1, fp);
                fclose(fp);
            } else if (!dump_missing)
                warning("Unable to open %s for writing, please make sure your NVR folder is writable\n", flash_path);
        } else
            fatal("Attempting to open the Flash file for writing with an empty invalid name\n");
    }

    free(dev->array);
    dev->array = NULL;

    free(dev);
}

const device_t catalyst_flash_device = {
    .name          = "Catalyst 28F010-D Flash BIOS",
    .internal_name = "catalyst_flash",
    .flags         = DEVICE_PCI,
    .local         = 0x31b4,
    .init          = catalyst_flash_init,
    .close         = catalyst_flash_close,
    .reset         = catalyst_flash_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t amd_am28f010_flash_device = {
    .name          = "AMD Am28F010-D Flash BIOS",
    .internal_name = "amd_am28f010_flash",
    .flags         = DEVICE_PCI,
    .local         = 0x01a7,
    .init          = catalyst_flash_init,
    .close         = catalyst_flash_close,
    .reset         = catalyst_flash_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t intel_28f010_flash_device = {
    .name          = "Intel 28F010 Flash BIOS",
    .internal_name = "intel_28f010_flash",
    .flags         = DEVICE_PCI,
    .local         = 0x89b4,
    .init          = catalyst_flash_init,
    .close         = catalyst_flash_close,
    .reset         = catalyst_flash_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

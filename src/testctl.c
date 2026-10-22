/*
 * Test control for automated firmware tests. When $BOX86_TESTCTL names a FIFO, each line written to
 * it is one command, run in the emulation thread between CPU blocks:
 *   key <hex>...   press and release each key (set 1 scan codes, 0x1xx = E0 prefixed)
 *   combo <hex>... press the keys together, then release them (e.g. 1d 38 53: Ctrl+Alt+Del)
 *   usbkey <hex>.. type on the USB keyboard: modifiers << 8 | HID usage each (e.g. 4c: Del, 054c: Ctrl+Alt+Del)
 *   text <file>    write the 80x25 text screen at B8000 to <file> (atomic rename)
 *   pci <file>     write the config space of every bus 0 PCI function to <file> (atomic rename)
 *   out <port> <v> write byte <v> to I/O <port> (hex)
 *   in <port> <f>  append "<port> <value>" (hex) read from I/O <port> to file <f>
 *   shot           save a PNG of the screen in <vm path>/screenshots/ at the next frame
 *   cr0 <file>     append CR0 (hex) to <file>
 *   flash <file>   append "<instructions fetched> <bytes read>" from the BIOS flash since the last flash command
 *   rtcadd <s>     advance the RTC by <s> seconds (decimal)
 *   reset          hard reset (reset button)
 *   quit           power off (NVR saved)
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/keyboard.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/nvr.h>
#include <86box/plat.h>
#include <86box/thread.h>
#include <86box/video.h>
#include <86box/device.h>
#include <86box/usb_uhci.h>
#include <86box/machine.h>
#include "cpu.h"

#define CMD_MAX 1024

static mutex_t *testctl_mutex;
static char     pending[CMD_MAX];

static void
testctl_thread(void *priv)
{
    const char *path = priv;
    char        line[CMD_MAX];

    for (;;) {
        FILE *fp = fopen(path, "r");
        if (!fp) {
            pclog("testctl: cannot open %s\n", path);
            return;
        }
        while (fgets(line, sizeof(line), fp)) {
            line[strcspn(line, "\r\n")] = 0;
            for (int queued = 0; !queued; plat_delay_ms(1)) {
                thread_wait_mutex(testctl_mutex);
                queued = !pending[0];
                if (queued)
                    strcpy(pending, line);
                thread_release_mutex(testctl_mutex);
            }
        }
        fclose(fp);
    }
}

static FILE *
open_tmp(const char *path, char *tmp)
{
    snprintf(tmp, CMD_MAX + 8, "%s.tmp", path);
    FILE *fp = fopen(tmp, "w");
    if (!fp)
        pclog("testctl: cannot write %s\n", tmp);
    return fp;
}

static void
pci_dump(const char *path)
{
    char     tmp[CMD_MAX + 8];
    FILE    *fp  = open_tmp(path, tmp);
    uint32_t cf8 = inl(0xcf8);

    if (!fp)
        return;
    for (int devfn = 0; devfn < 256; devfn++) {
        outl(0xcf8, 0x80000000 | (devfn << 8));
        if ((inl(0xcfc) & 0xffff) == 0xffff)
            continue;
        outl(0xcf8, 0x80000000 | ((devfn & ~7) << 8) | 0x0c);
        if ((devfn & 7) && !(inl(0xcfc) & 0x00800000))   /* functions 1-7 only on multi-function devices */
            continue;
        for (int reg = 0; reg < 256; reg += 4) {
            outl(0xcf8, 0x80000000 | (devfn << 8) | reg);
            uint32_t val = inl(0xcfc);
            if (!(reg & 0x0f))
                fprintf(fp, "%02x:%02x.%x %02x:", 0, devfn >> 3, devfn & 7, reg);
            fprintf(fp, " %02x %02x %02x %02x", val & 0xff, (val >> 8) & 0xff, (val >> 16) & 0xff, val >> 24);
            if ((reg & 0x0f) == 0x0c)
                fputc('\n', fp);
        }
    }
    outl(0xcf8, cf8);
    fclose(fp);
    rename(tmp, path);
}

void
text_dump(const char *path)
{
    char tmp[CMD_MAX + 8];
    FILE *fp = open_tmp(path, tmp);
    if (!fp)
        return;
    for (int row = 0; row < 25; row++) {
        for (int col = 0; col < 80; col++) {
            uint8_t c = mem_readb_phys(0xb8000 + 2 * (row * 80 + col));
            fputc((c >= 0x20 && c < 0x7f) ? c : ' ', fp);
        }
        fputc('\n', fp);
    }
    fclose(fp);
    rename(tmp, path);
}

/* BOX86_LIST_VIDEO=<file>: the video cards (bus cards) valid on the configured machine, one per line
   "<internal name>|<isa|pci>|<mono|color>|<config section>|<memory option: default, then each value=description>",
   then exit. */
static void
list_video(const char *path)
{
    FILE *fp = fopen(path, "w");
    if (!fp) {
        pclog("testctl: cannot write %s\n", path);
        exit(1);
    }
    for (int c = 2; video_card_getdevice(c); c++) {
        const device_t *dev = video_card_getdevice(c);
        if (!video_card_available(c) || !device_is_valid(dev, machine) || !(dev->flags & DEVICE_BUS))
            continue;   /* not a bus card: "external" (no device), the HD44780 LCD (parallel port) */
        fprintf(fp, "%s|%s|%s|%s|", dev->internal_name, (dev->flags & DEVICE_PCI) ? "pci" : "isa",
                (video_card_get_flags(c) & VIDEO_FLAG_TYPE_MASK) == VIDEO_FLAG_TYPE_MDA ? "mono" : "color", dev->name);
        for (const device_config_t *cfg = dev->config; cfg && cfg->type != CONFIG_END; cfg++) {
            if (strcmp(cfg->name, "memory") || cfg->type != CONFIG_SELECTION)
                continue;
            fprintf(fp, "%d", cfg->default_int);
            for (const device_config_selection_t *sel = cfg->selection; sel->description && sel->description[0]; sel++)
                fprintf(fp, ",%d=%s", sel->value, sel->description);
        }
        fputc('\n', fp);
    }
    fclose(fp);
    exit(0);
}

void
testctl_init(void)
{
    const char *path = getenv("BOX86_TESTCTL");
    if (getenv("BOX86_LIST_VIDEO"))
        list_video(getenv("BOX86_LIST_VIDEO"));
    if (!path)
        return;
    testctl_mutex = thread_create_mutex();
    thread_create(testctl_thread, strdup(path));
}

void
testctl_poll(void)
{
    char cmd[CMD_MAX];

    if (!testctl_mutex)
        return;
    thread_wait_mutex(testctl_mutex);
    strcpy(cmd, pending);
    pending[0] = 0;
    thread_release_mutex(testctl_mutex);
    if (!cmd[0])
        return;

    char *arg = strchr(cmd, ' ');
    if (arg)
        *arg++ = 0;
    else
        arg = cmd + strlen(cmd);

    if (!strcmp(cmd, "key")) {
        char *end;
        for (unsigned long scan = strtoul(arg, &end, 16); end != arg; scan = strtoul(arg, &end, 16)) {
            keyboard_input(1, scan);
            keyboard_input(0, scan);
            arg = end;
        }
    } else if (!strcmp(cmd, "combo")) {
        unsigned long keys[8];
        int n = 0;
        char *end;
        for (unsigned long scan = strtoul(arg, &end, 16); end != arg && n < 8; scan = strtoul(arg, &end, 16)) {
            keyboard_input(1, keys[n++] = scan);
            arg = end;
        }
        while (n > 0)
            keyboard_input(0, keys[--n]);
    } else if (!strcmp(cmd, "usbkey")) {
        char *end;
        for (unsigned long key = strtoul(arg, &end, 16); end != arg; key = strtoul(arg, &end, 16)) {
            usb_uhci_key(key);
            arg = end;
        }
    } else if (!strcmp(cmd, "text"))
        text_dump(arg);
    else if (!strcmp(cmd, "pci"))
        pci_dump(arg);
    else if (!strcmp(cmd, "out")) {
        unsigned port, val;
        if (sscanf(arg, "%x %x", &port, &val) == 2)
            outb(port, val);
    } else if (!strcmp(cmd, "in")) {
        unsigned port;
        char     file[CMD_MAX];
        FILE    *fp;
        if (sscanf(arg, "%x %1023s", &port, file) == 2 && (fp = fopen(file, "a"))) {
            fprintf(fp, "%03x %02x\n", port, inb(port));
            fclose(fp);
        }
    }
    else if (!strcmp(cmd, "cr0")) {
        FILE *fp = fopen(arg, "a");
        if (fp) {
            fprintf(fp, "%08x\n", cr0);
            fclose(fp);
        }
    } else if (!strcmp(cmd, "flash")) {
        FILE *fp = fopen(arg, "a");
        if (fp) {
            fprintf(fp, "%llu %llu\n", (unsigned long long) flash_fetch_count, (unsigned long long) flash_read_bytes);
            fclose(fp);
        }
        flash_fetch_count = flash_read_bytes = 0;
    } else if (!strcmp(cmd, "rtcadd")) {
        for (int s = atoi(arg); s > 0; s--)
            rtc_tick();
    } else if (!strcmp(cmd, "shot"))
        ++monitors[0].mon_screenshots_raw;
    else if (!strcmp(cmd, "reset"))
        pc_reset_hard();
    else if (!strcmp(cmd, "quit"))
        plat_power_off();
    else
        pclog("testctl: unknown command '%s'\n", cmd);
}

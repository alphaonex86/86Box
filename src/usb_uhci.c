/*
 * The MB-8433UUD-A's USB card for firmware tests: a VIA VT6212, PCI functions 0 and 1 UHCI (1106:3038 rev 61, two
 * ports each) and 2 EHCI (1106:3104 rev 63, the same four ports; register values read on the operator's card).
 * EHCI owns a port once CONFIGFLAG is set, unless its PORT_OWNER hands it to the companion UHCI (a low or full speed
 * device: no transaction translator). Schedules are walked from RAM by bus mastering every ms, as the hardware does
 * (nothing runs while PCI bus mastering is off). Port 0: a low-speed HID boot keyboard (BOX86_USB_KBD=1; keys typed
 * by the test control's "usbkey"; its TDs need the LS bit, else they time out); port 1: a bulk-only mass storage
 * disk on an image file (BOX86_USB_DISK=path), high speed on EHCI, full speed on UHCI. BOX86_USB_FAULT injects one
 * fault: kbd-stall (the keyboard stalls its control requests), disk-nak (the disk never sends data), disk-stall (it
 * stalls the data of READ), disk-csw (bad CSW signature), disk-gone (it is unplugged 3 s after its first READ).
 * Enough of USB for a BIOS and an OS driver: control, interrupt and bulk transfers, SCSI INQUIRY, TEST UNIT READY,
 * REQUEST SENSE, READ CAPACITY, READ/WRITE(10), MODE SENSE.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/dma.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/pci.h>
#include <86box/postcard.h>
#include <86box/timer.h>
#include <86box/usb_uhci.h>

enum { R_NAK = -2, R_STALL = -1 };
enum { F_NONE, F_KBD_STALL, F_DISK_NAK, F_DISK_STALL, F_DISK_CSW, F_DISK_GONE };
static int Fault;
static uint64_t GoneAt;                                /* disk-gone: the frame count it disappears at */
static uint64_t Frames;

typedef struct usbdev_t usbdev_t;
struct usbdev_t {
    int present, low_speed, high_speed, addr, new_addr, config;   /* high_speed: running at 480 Mbit/s (EHCI) */
    const uint8_t *dev_desc, *cfg_desc, *hs_cfg_desc;             /* hs_cfg_desc: at high speed, if it can */
    int cfg_len;
    int (*class_request)(usbdev_t *d, const uint8_t *setup, uint8_t *data);   /* data length, or R_STALL */
    int (*in)(usbdev_t *d, int ep, uint8_t *buf, int len);
    int (*out)(usbdev_t *d, int ep, const uint8_t *buf, int len);
    uint8_t setup[8];
    uint8_t ctl[512];                                  /* control data stage */
    int     ctl_len, ctl_pos, ctl_in;
};

/****************************************************************
 * Keyboard (HID boot protocol)
 ****************************************************************/

static const uint8_t kbd_dev_desc[18] = { 18, 1, 0x10, 0x01, 0, 0, 0, 8, 0x34, 0x12, 0x01, 0x00, 0x00, 0x01, 0, 0, 0, 1 };
static const uint8_t kbd_report_desc[63] = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
    0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01,
    0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xc0
};
static const uint8_t kbd_cfg_desc[34] = {
    9, 2, 34, 0, 1, 1, 0, 0xa0, 50,
    9, 4, 0, 0, 1, 3, 1, 1, 0,                          /* HID, boot interface, keyboard */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(kbd_report_desc), 0,
    7, 5, 0x81, 3, 8, 0, 10                             /* interrupt IN 1, 8 bytes, 10 ms */
};

#define KBD_QUEUE 64
static uint8_t kbd_reports[KBD_QUEUE][8];
static int     kbd_head, kbd_tail;

static void
kbd_push(uint8_t mods, uint8_t usage)
{
    uint8_t *r = kbd_reports[kbd_head];
    memset(r, 0, 8);
    r[0] = mods;
    r[2] = usage;
    if (((kbd_head + 1) % KBD_QUEUE) != kbd_tail)
        kbd_head = (kbd_head + 1) % KBD_QUEUE;
}

/* "usbkey" of the test control: modifiers << 8 | usage, pressed then released. */
void
usb_uhci_key(int mods_usage)
{
    kbd_push(mods_usage >> 8, mods_usage & 0xff);
    kbd_push(0, 0);
}

static int
kbd_class(usbdev_t *d, const uint8_t *setup, uint8_t *data)
{
    if (setup[0] == 0x81 && setup[1] == 6 && setup[3] == 0x22) {          /* report descriptor */
        memcpy(data, kbd_report_desc, sizeof(kbd_report_desc));
        return sizeof(kbd_report_desc);
    }
    if (setup[0] == 0x21 && (setup[1] == 0x09 || setup[1] == 0x0a || setup[1] == 0x0b))
        return 0;                                       /* SET_REPORT (LEDs), SET_IDLE, SET_PROTOCOL */
    if (setup[0] == 0xa1 && setup[1] == 0x01) {        /* GET_REPORT */
        memset(data, 0, 8);
        return 8;
    }
    return R_STALL;
}

static int
kbd_in(usbdev_t *d, int ep, uint8_t *buf, int len)
{
    if ((ep != 1) || (kbd_head == kbd_tail))
        return R_NAK;
    memcpy(buf, kbd_reports[kbd_tail], 8);
    kbd_tail = (kbd_tail + 1) % KBD_QUEUE;
    return len < 8 ? len : 8;
}

static int
no_out(usbdev_t *d, int ep, const uint8_t *buf, int len)
{
    return R_STALL;
}

/****************************************************************
 * Mass storage (bulk-only transport, SCSI)
 ****************************************************************/

static const uint8_t msc_dev_desc[18] = { 18, 1, 0x00, 0x02, 0, 0, 0, 64, 0x81, 0x07, 0x67, 0x55, 0x00, 0x01, 0, 0, 0, 1 };
static const uint8_t msc_cfg_desc[32] = {
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,
    9, 4, 0, 0, 2, 8, 6, 0x50, 0,                       /* mass storage, SCSI transparent, bulk-only */
    7, 5, 0x81, 2, 64, 0, 0,                            /* bulk IN 1 */
    7, 5, 0x02, 2, 64, 0, 0                             /* bulk OUT 2 */
};
static const uint8_t msc_hs_cfg_desc[32] = {
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,
    9, 4, 0, 0, 2, 8, 6, 0x50, 0,
    7, 5, 0x81, 2, 0, 2, 0,                             /* 512-byte packets at high speed */
    7, 5, 0x02, 2, 0, 2, 0
};

static struct {
    FILE    *img;
    uint32_t blocks;
    enum { M_CBW, M_DATA_IN, M_DATA_OUT, M_CSW } state;
    uint8_t  cbw[31], csw[13];
    uint8_t *data;
    uint32_t data_len, data_pos, lba;
    uint8_t  sense_key, asc;
} Msc;

static void
msc_csw(uint8_t status, uint32_t residue)
{
    memcpy(Msc.csw, "USBS", 4);
    memcpy(&Msc.csw[4], &Msc.cbw[4], 4);                /* tag */
    memcpy(&Msc.csw[8], &residue, 4);
    Msc.csw[12] = status;
    Msc.state   = M_CSW;
}

static uint32_t
be32(const uint8_t *p)
{
    return (uint32_t) p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}

/* A command block: its data phase, or straight the CSW. */
static void
msc_command(void)
{
    const uint8_t *cb = &Msc.cbw[15];
    uint32_t       want;
    uint8_t        reply[36];
    int            n = -1;

    memcpy(&want, &Msc.cbw[8], 4);
    Msc.data_pos = 0;
    memset(reply, 0, sizeof(reply));
    switch (cb[0]) {
        case 0x00:                                      /* TEST UNIT READY */
        case 0x1b:                                      /* START STOP UNIT */
        case 0x1e:                                      /* PREVENT ALLOW MEDIUM REMOVAL */
            n = 0;
            break;
        case 0x03:                                      /* REQUEST SENSE */
            reply[0] = 0x70, reply[2] = Msc.sense_key, reply[7] = 10, reply[12] = Msc.asc;
            Msc.sense_key = Msc.asc = 0;
            n = 18;
            break;
        case 0x12:                                      /* INQUIRY */
            reply[1] = 0x80, reply[2] = 2, reply[3] = 2, reply[4] = 31;
            memcpy(&reply[8], "86Box   USB DISK        1.00", 28);
            n = 36;
            break;
        case 0x25: {                                    /* READ CAPACITY(10) */
            uint32_t last = Msc.blocks - 1, bs = 512;
            for (int i = 0; i < 4; i++)
                reply[i] = last >> (24 - 8 * i), reply[4 + i] = bs >> (24 - 8 * i);
            n = 8;
            break;
        }
        case 0x1a:                                      /* MODE SENSE(6) */
            reply[0] = 3;
            n = 4;
            break;
        case 0x5a:                                      /* MODE SENSE(10) */
            reply[1] = 6;
            n = 8;
            break;
        case 0x28:                                      /* READ(10) */
            if (Fault == F_DISK_GONE && !GoneAt)
                GoneAt = Frames + 3000;
            /* fall through */
        case 0x2a: {                                    /* WRITE(10) */
            uint32_t count = cb[7] << 8 | cb[8];
            Msc.lba        = be32(&cb[2]);
            if (((uint64_t) Msc.lba + count > Msc.blocks) || (count * 512 != want)) {
                Msc.sense_key = 5, Msc.asc = 0x21;      /* illegal request, LBA out of range */
                msc_csw(1, want);
                return;
            }
            free(Msc.data);
            Msc.data     = calloc(1, count * 512 + 1);
            Msc.data_len = count * 512;
            if (cb[0] == 0x28) {
                fseek(Msc.img, (long) Msc.lba * 512, SEEK_SET);
                if (fread(Msc.data, 512, count, Msc.img) != count)
                    Msc.sense_key = 3, Msc.asc = 0x11;  /* unrecovered read error: the data stays zero */
                Msc.state = count ? M_DATA_IN : M_CSW;
            } else
                Msc.state = count ? M_DATA_OUT : M_CSW;
            if (!count)
                msc_csw(0, 0);
            return;
        }
        default:
            Msc.sense_key = 5, Msc.asc = 0x20;          /* illegal request, invalid command */
            msc_csw(1, want);
            return;
    }
    if (n > (int) want)
        n = want;
    free(Msc.data);
    Msc.data = malloc(n + 1);
    memcpy(Msc.data, reply, n);
    Msc.data_len = n;
    if (n && (Msc.cbw[12] & 0x80))
        Msc.state = M_DATA_IN;
    else
        msc_csw(0, want - n);
}

static int
msc_class(usbdev_t *d, const uint8_t *setup, uint8_t *data)
{
    if (setup[0] == 0xa1 && setup[1] == 0xfe) {        /* GET MAX LUN */
        data[0] = 0;
        return 1;
    }
    if (setup[0] == 0x21 && setup[1] == 0xff) {        /* bulk-only mass storage reset */
        Msc.state = M_CBW;
        return 0;
    }
    return R_STALL;
}

static int
msc_out(usbdev_t *d, int ep, const uint8_t *buf, int len)
{
    if (ep != 2)
        return R_STALL;
    if (Msc.state == M_CBW) {
        if ((len != 31) || memcmp(buf, "USBC", 4))
            return R_STALL;
        memcpy(Msc.cbw, buf, 31);
        msc_command();
        return len;
    }
    if (Msc.state != M_DATA_OUT)
        return R_STALL;
    if ((uint32_t) len > Msc.data_len - Msc.data_pos)
        len = Msc.data_len - Msc.data_pos;
    memcpy(Msc.data + Msc.data_pos, buf, len);
    Msc.data_pos += len;
    if (Msc.data_pos == Msc.data_len) {
        fseek(Msc.img, (long) Msc.lba * 512, SEEK_SET);
        if (fwrite(Msc.data, 1, Msc.data_len, Msc.img) != Msc.data_len)
            Msc.sense_key = 3, Msc.asc = 0x0c;          /* write error */
        fflush(Msc.img);
        msc_csw(Msc.sense_key ? 1 : 0, 0);
    }
    return len;
}

static int
msc_in(usbdev_t *d, int ep, uint8_t *buf, int len)
{
    if (ep != 1)
        return R_STALL;
    if (Fault == F_DISK_NAK)
        return R_NAK;
    if (Fault == F_DISK_STALL && Msc.state == M_DATA_IN && Msc.cbw[15] == 0x28)
        return R_STALL;
    if (Msc.state == M_DATA_IN) {
        if ((uint32_t) len > Msc.data_len - Msc.data_pos)
            len = Msc.data_len - Msc.data_pos;
        memcpy(buf, Msc.data + Msc.data_pos, len);
        Msc.data_pos += len;
        if (Msc.data_pos == Msc.data_len) {
            uint32_t want;
            memcpy(&want, &Msc.cbw[8], 4);
            msc_csw(Msc.sense_key ? 1 : 0, want - Msc.data_len);
        }
        return len;
    }
    if (Msc.state == M_CSW) {
        memcpy(buf, Msc.csw, 13);
        if (Fault == F_DISK_CSW)
            buf[0] = 'X';
        Msc.state = M_CBW;
        return 13;
    }
    return R_NAK;
}

/****************************************************************
 * Control transfers (endpoint 0)
 ****************************************************************/

static int
dev_setup(usbdev_t *d, const uint8_t *setup)
{
    if (Fault == F_KBD_STALL && d->class_request == kbd_class)
        return R_STALL;
    uint16_t value = setup[2] | setup[3] << 8, length = setup[6] | setup[7] << 8;
    int      n     = 0;

    memcpy(d->setup, setup, 8);
    d->ctl_in  = setup[0] & 0x80;
    d->ctl_pos = 0;
    if ((setup[0] & 0x60) == 0) {                       /* standard requests */
        switch (setup[1]) {
            case 0:                                     /* GET_STATUS */
                memset(d->ctl, 0, 2);
                n = 2;
                break;
            case 5:                                     /* SET_ADDRESS: once the status stage is done */
                d->new_addr = value & 0x7f;
                break;
            case 6:                                     /* GET_DESCRIPTOR */
                if ((value >> 8) == 1)
                    memcpy(d->ctl, d->dev_desc, n = 18);
                else if ((value >> 8) == 2)
                    memcpy(d->ctl, d->high_speed && d->hs_cfg_desc ? d->hs_cfg_desc : d->cfg_desc, n = d->cfg_len);
                else if ((value >> 8) == 0x22 && d->class_request)
                    n = d->class_request(d, (const uint8_t[8]) { 0x81, 6, 0, 0x22 }, d->ctl);
                else
                    n = R_STALL;
                break;
            case 8:                                     /* GET_CONFIGURATION */
                d->ctl[0] = d->config;
                n         = 1;
                break;
            case 9:                                     /* SET_CONFIGURATION */
                d->config = value & 0xff;
                break;
            case 1:                                     /* CLEAR_FEATURE (endpoint halt) */
            case 3:                                     /* SET_FEATURE */
            case 11:                                    /* SET_INTERFACE */
                break;
            default:
                n = R_STALL;
        }
    } else
        n = d->class_request(d, setup, d->ctl);
    if (n < 0)
        return R_STALL;
    d->ctl_len = n < length ? n : length;
    return 8;
}

static int
dev_in(usbdev_t *d, int ep, uint8_t *buf, int len)
{
    if (ep)
        return d->in(d, ep, buf, len);
    if (!d->ctl_in) {                                   /* status stage of an OUT or no-data request */
        if (d->setup[1] == 5 && (d->setup[0] & 0x60) == 0)
            d->addr = d->new_addr;
        return 0;
    }
    int n = d->ctl_len - d->ctl_pos;
    if (n > len)
        n = len;
    memcpy(buf, d->ctl + d->ctl_pos, n);
    d->ctl_pos += n;
    return n;
}

static int
dev_out(usbdev_t *d, int ep, const uint8_t *buf, int len)
{
    if (ep)
        return d->out(d, ep, buf, len);
    return len;                                         /* data or status stage: accepted */
}


/****************************************************************
 * The card: VIA VT6212, PCI functions 0 and 1 UHCI (2 ports each), 2 EHCI (the same 4 ports)
 ****************************************************************/

#define NPORTS 4
static usbdev_t Dev[NPORTS];                           /* by port: 0 keyboard, 1 disk */

enum { CMD_RS = 1, CMD_HCRESET = 2, CMD_GRESET = 4 };
enum { STS_USBINT = 1, STS_ERROR = 2, STS_HALTED = 0x20 };
enum { PORT_CCS = 1, PORT_CSC = 2, PORT_PE = 4, PORT_PEC = 8, PORT_LSDA = 0x100, PORT_RESET = 0x200 };
enum { TD_ACTIVE = 1 << 23, TD_STALLED = 1 << 22, TD_NAK = 1 << 19, TD_TIMEOUT = 1 << 18, TD_IOC = 1 << 24,
       TD_LS = 1 << 26, TD_SPD = 1 << 29 };

enum { E_RS = 1, E_HCRESET = 2, E_PSE = 0x10, E_ASE = 0x20, E_IAAD = 0x40 };
enum { E_USBINT = 1, E_USBERRINT = 2, E_PCD = 4, E_FLR = 8, E_IAA = 0x20, E_HALTED = 0x1000, E_PSS = 0x4000,
       E_ASS = 0x8000 };
enum { EP_CCS = 1, EP_CSC = 2, EP_PE = 4, EP_PEC = 8, EP_OCC = 0x20, EP_PR = 0x100, EP_LS_K = 0x400, EP_LS_J = 0x800,
       EP_PP = 0x1000, EP_PO = 0x2000 };
enum { Q_PING = 1, Q_XACTERR = 8, Q_HALTED = 0x40, Q_ACTIVE = 0x80, Q_IOC = 0x8000, Q_TOGGLE = 1u << 31 };

typedef struct uhci_t {
    uint8_t  pci[256];
    uint8_t  irq_state, pin;
    int      first;                                    /* its first port */
    uint16_t io_base;
    uint16_t cmd, sts, intr, frnum;
    uint32_t flbase;
    uint8_t  sof;
    uint16_t portsc[2];
} uhci_t;

typedef struct ehci_t {
    uint8_t       pci[256];
    uint8_t       irq_state;
    uint32_t      cmd, sts, intr, frindex, plbase, asyncbase, configflag;
    uint32_t      portsc[NPORTS];                      /* PE, PR, PO and the change bits; the rest is computed */
    mem_mapping_t mmio;
} ehci_t;

typedef struct card_t {
    uint8_t    slot;
    int        dma_on;                                 /* a schedule run with bus mastering, as last logged */
    uhci_t     uhci[2];
    ehci_t     ehci;
    pc_timer_t frame_timer;
} card_t;

static card_t *Card;

static uint32_t
rd32(uint32_t addr)
{
    uint32_t v;
    dma_bm_read(addr, (uint8_t *) &v, 4, 4);
    return v;
}

static void
wr32(uint32_t addr, uint32_t v)
{
    dma_bm_write(addr, (uint8_t *) &v, 4, 4);
}

/* Schedules run with bus mastering: changes go to the POST log (tests: no USB DMA may outlive a reboot) */
static void
dma_check(card_t *c)
{
    int on = ((c->ehci.cmd & E_RS) && (c->ehci.pci[0x04] & 4));
    for (int f = 0; f < 2; f++)
        on |= (c->uhci[f].cmd & CMD_RS) && (c->uhci[f].pci[0x04] & 4);
    if (c->dma_on != on) {
        c->dma_on = on;
        box86_postlog("USB-DMA %s", on ? "on" : "off");
    }
}

static int
ehci_owns(card_t *c, int p)
{
    return c->ehci.configflag && !(c->ehci.portsc[p] & EP_PO);
}

/* The port changed owner (EHCI or its companion UHCI): the device is disconnected from one, connected to the other. */
static void
port_route(card_t *c, int p)
{
    usbdev_t *d   = &Dev[p];
    uint16_t *usc = &c->uhci[p / 2].portsc[p % 2];

    d->addr = d->config = d->high_speed = 0;
    c->ehci.portsc[p] = (c->ehci.portsc[p] & ~(EP_PE | EP_PR)) | (d->present ? EP_CSC : 0);
    if (d->present && !ehci_owns(c, p))
        *usc = PORT_CCS | PORT_CSC | (d->low_speed ? PORT_LSDA : 0);
    else
        *usc = d->present ? PORT_CSC : 0;
}

/****************************************************************
 * UHCI
 ****************************************************************/

static void
uhci_irq(card_t *c, uhci_t *u)
{
    if ((u->sts & (STS_USBINT | STS_ERROR)) && (u->intr & 0x0d))
        pci_set_irq(c->slot, u->pin, &u->irq_state);
    else
        pci_clear_irq(c->slot, u->pin, &u->irq_state);
}

static usbdev_t *
uhci_find(card_t *c, uhci_t *u, int addr)
{
    for (int i = 0; i < 2; i++) {
        int p = u->first + i;
        if (Dev[p].present && !ehci_owns(c, p) && (u->portsc[i] & PORT_PE) && !(u->portsc[i] & PORT_RESET)
            && Dev[p].addr == addr)
            return &Dev[p];
    }
    return NULL;
}

/* One TD: 1 done, 0 NAK or not active (a queue head stays on it: Linux appends to its inactive dummy TD), -1 error
   (stalled or no device). */
static int
uhci_td(card_t *c, uhci_t *u, uint32_t td, uint32_t *link)
{
    uint32_t status = rd32(td + 4), token = rd32(td + 8), buffer = rd32(td + 12);
    *link = rd32(td);
    if (!(status & TD_ACTIVE))
        return 0;
    int       pid = token & 0xff, addr = (token >> 8) & 0x7f, ep = (token >> 15) & 0xf;
    int       maxlen = ((token >> 21) + 1) & 0x7ff;
    uint8_t   buf[1024];
    usbdev_t *d = uhci_find(c, u, addr);
    int       n;

    if (!d || (d->low_speed && !(status & TD_LS)))   /* a full-speed packet: a low-speed device ignores it */
        n = -3;
    else if (pid == 0x2d) {                             /* SETUP */
        dma_bm_read(buffer, buf, 8, 4);
        n = dev_setup(d, buf);
    } else if (pid == 0xe1) {                           /* OUT */
        dma_bm_read(buffer, buf, maxlen, 4);
        n = dev_out(d, ep, buf, maxlen);
    } else if (pid == 0x69) {                           /* IN */
        n = dev_in(d, ep, buf, maxlen);
        if (n > 0)
            dma_bm_write(buffer, buf, n, 4);
    } else
        n = R_STALL;
    if (n == R_NAK) {
        wr32(td + 4, status | TD_NAK);
        return 0;
    }
    status &= ~(TD_ACTIVE | TD_NAK | 0x7ff);
    if (n == -3)
        status |= TD_TIMEOUT | TD_STALLED;
    else if (n < 0)
        status |= TD_STALLED;
    else
        status |= (n - 1) & 0x7ff;                      /* actual length - 1, 7FFh for none */
    wr32(td + 4, status);
    if (status & TD_IOC)
        u->sts |= STS_USBINT;
    if (n < 0) {
        u->sts |= STS_ERROR;
        return -1;
    }
    if (pid == 0x69 && n < maxlen && (status & TD_SPD)) {   /* short packet: the queue head stays on this TD */
        u->sts |= STS_USBINT;
        return 0;
    }
    return 1;
}

static void
uhci_frame(card_t *c, uhci_t *u)
{
    uint32_t link, next;

    if (!(u->cmd & CMD_RS) || !(u->pci[0x04] & 4))     /* stopped, or no bus mastering */
        return;
    link = rd32(u->flbase + (u->frnum & 0x3ff) * 4);
    for (int i = 0; i < 256 && !(link & 1); i++) {
        uint32_t addr = link & ~0xf;
        if (link & 2) {                                 /* queue head: its TDs until one is not done */
            uint32_t elem = rd32(addr + 4);
            for (int j = 0; j < 64 && !(elem & 3); j++) {
                if (uhci_td(c, u, elem & ~0xf, &next) <= 0)
                    break;
                elem = next;
                wr32(addr + 4, elem);
                if (!(next & 4))                        /* breadth first: one TD per queue per pass */
                    break;
            }
            link = rd32(addr);
        } else {
            uhci_td(c, u, addr, &next);
            link = next;
        }
    }
    u->frnum = (u->frnum + 1) & 0x7ff;
    uhci_irq(c, u);
}

static void
uhci_port_reset(card_t *c, uhci_t *u)
{
    for (int i = 0; i < 2; i++) {
        int p = u->first + i;
        if (ehci_owns(c, p))
            u->portsc[i] = 0;
        else {
            u->portsc[i] = Dev[p].present ? PORT_CCS | PORT_CSC | (Dev[p].low_speed ? PORT_LSDA : 0) : 0;
            Dev[p].addr = Dev[p].config = 0;
        }
    }
}

static void
uhci_reset(card_t *c, uhci_t *u)
{
    u->cmd = u->intr = u->frnum = 0;
    u->sts    = STS_HALTED;
    u->flbase = 0;
    u->sof    = 0x40;
    uhci_port_reset(c, u);
    dma_check(c);
}

static uhci_t *
uhci_of(uint16_t port)
{
    return (Card->uhci[1].io_base && (port & ~0x1f) == Card->uhci[1].io_base) ? &Card->uhci[1] : &Card->uhci[0];
}

static uint16_t
uhci_readw(uint16_t port, void *priv)
{
    uhci_t *u = uhci_of(port);
    switch (port - u->io_base) {
        case 0x00: return u->cmd;
        case 0x02: return u->sts;
        case 0x04: return u->intr;
        case 0x06: return u->frnum;
        case 0x08: return u->flbase & 0xffff;
        case 0x0a: return u->flbase >> 16;
        case 0x0c: return u->sof;
        case 0x10: return u->portsc[0] | 0x80;
        case 0x12: return u->portsc[1] | 0x80;
    }
    return 0xffff;
}

static uint8_t
uhci_readb(uint16_t port, void *priv)
{
    return uhci_readw(port & ~1, priv) >> (8 * (port & 1));
}

static uint32_t
uhci_readl(uint16_t port, void *priv)
{
    return uhci_readw(port, priv) | (uint32_t) uhci_readw(port + 2, priv) << 16;
}

static void
uhci_writew(uint16_t port, uint16_t val, void *priv)
{
    card_t *c = (card_t *) priv;
    uhci_t *u = uhci_of(port);
    switch (port - u->io_base) {
        case 0x00:
            if (val & (CMD_HCRESET | CMD_GRESET)) {
                uhci_reset(c, u);
                break;
            }
            u->cmd = val;
            u->sts = (u->sts & ~STS_HALTED) | (val & CMD_RS ? 0 : STS_HALTED);
            dma_check(c);
            break;
        case 0x02: u->sts &= ~(val & 0x1f); uhci_irq(c, u); break;
        case 0x04: u->intr = val & 0x0f; uhci_irq(c, u); break;
        case 0x06: u->frnum = val & 0x7ff; break;
        case 0x08: u->flbase = (u->flbase & 0xffff0000) | (val & 0xf000); break;
        case 0x0a: u->flbase = (u->flbase & 0xffff) | (uint32_t) val << 16; break;
        case 0x0c: u->sof = val & 0x7f; break;
        case 0x10:
        case 0x12: {
            int       i  = (port - u->io_base - 0x10) / 2;
            uint16_t *sc = &u->portsc[i];
            if (val & PORT_RESET) {
                Dev[u->first + i].addr = Dev[u->first + i].config = 0;
                *sc = (*sc & ~PORT_PE) | PORT_RESET;
            } else
                *sc &= ~PORT_RESET;
            *sc &= ~(val & (PORT_CSC | PORT_PEC));      /* write 1 to clear */
            if ((val & PORT_PE) && (*sc & PORT_CCS) && !(*sc & PORT_RESET))
                *sc |= PORT_PE;
            else if (!(val & PORT_PE))
                *sc &= ~PORT_PE;
            break;
        }
    }
}

static void
uhci_writeb(uint16_t port, uint8_t val, void *priv)
{
    uint16_t old = uhci_readw(port & ~1, priv);
    uhci_writew(port & ~1, (port & 1) ? (old & 0xff) | val << 8 : (old & 0xff00) | val, priv);
}

static void
uhci_writel(uint16_t port, uint32_t val, void *priv)
{
    uhci_writew(port, val & 0xffff, priv);
    uhci_writew(port + 2, val >> 16, priv);
}

static void
uhci_io(card_t *c, uhci_t *u, int on)
{
    if (u->io_base)
        io_removehandler(u->io_base, 0x20, uhci_readb, uhci_readw, uhci_readl, uhci_writeb, uhci_writew, uhci_writel, c);
    u->io_base = (u->pci[0x20] | u->pci[0x21] << 8) & 0xffe0;
    if (on && u->io_base && (u->pci[0x04] & 1))
        io_sethandler(u->io_base, 0x20, uhci_readb, uhci_readw, uhci_readl, uhci_writeb, uhci_writew, uhci_writel, c);
    else
        u->io_base = 0;
}

static void
uhci_pci_write(card_t *c, uhci_t *u, int addr, uint8_t val)
{
    switch (addr) {
        case 0x04: u->pci[0x04] = val & 0x07; uhci_io(c, u, 1); dma_check(c); break;
        case 0x0d: u->pci[0x0d] = val; break;
        case 0x20: u->pci[0x20] = (val & 0xe0) | 1; uhci_io(c, u, 1); break;
        case 0x21: u->pci[0x21] = val; uhci_io(c, u, 1); break;
        case 0x3c: u->pci[0x3c] = val; break;
        case 0x40 ... 0x4f:
        case 0xc0 ... 0xc1: u->pci[addr] = val; break;
    }
}

static void
uhci_init_pci(uhci_t *u, int func)
{
    u->pci[0x00] = 0x06, u->pci[0x01] = 0x11, u->pci[0x02] = 0x38, u->pci[0x03] = 0x30;
    u->pci[0x06] = 0x10, u->pci[0x07] = 0x02, u->pci[0x08] = 0x61;
    u->pci[0x09] = 0x00, u->pci[0x0a] = 0x03, u->pci[0x0b] = 0x0c, u->pci[0x0e] = 0x80;
    u->pci[0x20] = 0x01, u->pci[0x60] = 0x10, u->pci[0xc1] = 0x20;
    u->pci[0x3d] = u->pin = func ? PCI_INTB : PCI_INTA;
    u->first     = 2 * func;
}

/****************************************************************
 * EHCI (root ports only, no transaction translator: a full or low speed device goes to the companion)
 ****************************************************************/

static void
ehci_irq(card_t *c)
{
    ehci_t *e = &c->ehci;
    if (e->sts & e->intr & 0x3f)
        pci_set_irq(c->slot, PCI_INTC, &e->irq_state);
    else
        pci_clear_irq(c->slot, PCI_INTC, &e->irq_state);
}

static usbdev_t *
ehci_find(card_t *c, int addr)
{
    for (int p = 0; p < NPORTS; p++)
        if (Dev[p].present && ehci_owns(c, p) && (c->ehci.portsc[p] & EP_PE) && Dev[p].addr == addr)
            return &Dev[p];
    return NULL;
}

/* The qTD in the overlay of queue head qh: 1 done, 0 NAK (stays active), -1 halted. */
static int
ehci_qtd(card_t *c, uint32_t qh)
{
    ehci_t   *e = &c->ehci;
    uint32_t  chars = rd32(qh + 4), token = rd32(qh + 0x18), cur = rd32(qh + 0x0c) & ~0x1f, buf[5];
    int       ep = (chars >> 8) & 0xf, maxp = (chars >> 16) & 0x7ff, pid = (token >> 8) & 3;
    int       total = (token >> 16) & 0x7fff, page = (token >> 12) & 7, done = 0, packets = 0, n = 0, shrt = 0;
    usbdev_t *d = ehci_find(c, chars & 0x7f);
    uint8_t   data[1024];

    for (int i = 0; i < 5; i++)
        buf[i] = rd32(qh + 0x1c + 4 * i);
    uint32_t off = buf[0] & 0xfff;
    if (!d || ((chars >> 12) & 3) != 2 || !maxp)       /* not a high-speed endpoint: no device answers */
        n = -3;
    else if (pid == 2) {                                /* SETUP */
        dma_bm_read((buf[page] & ~0xfff) + off, data, 8, 4);
        if ((n = dev_setup(d, data)) == 8)
            done = 8, total = 0, packets = 1;
    } else
        do {
            int chunk = total < maxp ? total : maxp, room = (5 - page) * 4096 - off;   /* 5 buffer pages at most */
            if (chunk > room)
                chunk = room;
            for (int i = 0; i < chunk; i++) {           /* bytes may cross into the next buffer page */
                uint32_t o = off + i;
                uint32_t a = (buf[page + (o >> 12)] & ~0xfff) + (o & 0xfff);
                if (pid == 0)
                    dma_bm_read(a, &data[i], 1, 1);
            }
            n = pid == 0 ? dev_out(d, ep, data, chunk) : dev_in(d, ep, data, chunk);
            if (n < 0)
                break;
            if (pid == 1)
                for (int i = 0; i < n; i++) {
                    uint32_t o = off + i;
                    dma_bm_write((buf[page + (o >> 12)] & ~0xfff) + (o & 0xfff), &data[i], 1, 1);
                }
            done += n, total -= n, packets++;
            page += (off + n) >> 12, off = (off + n) & 0xfff;
            shrt = pid == 1 && n < chunk;
        } while (total > 0 && !shrt && page < 5);
    if (n == R_NAK && !packets)
        return 0;
    token = (token & ~(0x7fff0000 | 0x7000)) | (uint32_t) total << 16 | (page & 7) << 12;
    if (packets & 1)
        token ^= Q_TOGGLE;
    if (n == R_NAK) {                                   /* some data moved, the rest later */
        wr32(qh + 0x18, token);
        wr32(qh + 0x1c, (buf[0] & ~0xfff) | off);
        return 0;
    }
    token &= ~Q_ACTIVE;
    if (n < 0) {
        token = (token | Q_HALTED | (n == -3 ? Q_XACTERR : 0)) & ~0xc00;   /* CERR 0 */
        e->sts |= E_USBERRINT;
    }
    if ((token & Q_IOC) || shrt)
        e->sts |= E_USBINT;
    wr32(cur + 8, token);
    wr32(qh + 0x18, token);
    wr32(qh + 0x1c, (buf[0] & ~0xfff) | off);
    if (shrt && !(rd32(qh + 0x14) & 1))                /* short packet: on to the alternate next qTD */
        wr32(qh + 0x10, rd32(qh + 0x14));
    return n < 0 ? -1 : 1;
}

/* A queue head: its qTDs in order, until one is not done or the queue is empty. */
static void
ehci_qh(card_t *c, uint32_t qh)
{
    for (int i = 0; i < 16; i++) {
        uint32_t token = rd32(qh + 0x18);
        if (token & Q_HALTED)
            return;
        if (!(token & Q_ACTIVE)) {
            uint32_t next = rd32(qh + 0x10), qtd[8];
            if (next & 1)
                return;
            next &= ~0x1f;
            for (int j = 0; j < 8; j++)
                qtd[j] = rd32(next + 4 * j);
            if (!(qtd[2] & Q_ACTIVE))                  /* Linux's inactive dummy at the end */
                return;
            if (!(rd32(qh + 4) & (1 << 14)))            /* DTC 0: the queue head keeps the data toggle */
                qtd[2] = (qtd[2] & ~Q_TOGGLE) | (token & Q_TOGGLE);
            wr32(qh + 0x0c, next);
            for (int j = 0; j < 8; j++)
                wr32(qh + 0x10 + 4 * j, qtd[j]);
        }
        if (ehci_qtd(c, qh) <= 0)
            return;
    }
}

static void
ehci_frame(card_t *c)
{
    ehci_t  *e    = &c->ehci;
    uint32_t size = 1024 >> ((e->cmd >> 2) & 3);

    if (!(e->cmd & E_RS))
        return;
    e->frindex = (e->frindex + 8) & 0x3fff;
    if (!((e->frindex >> 3) & (size - 1)))
        e->sts |= E_FLR;
    if (e->pci[0x04] & 4) {
        if (e->cmd & E_PSE) {
            uint32_t link = rd32((e->plbase & ~0xfff) + ((e->frindex >> 3) & (size - 1)) * 4);
            for (int i = 0; i < 64 && !(link & 1); i++) {
                if (((link >> 1) & 3) == 1)             /* queue head (no isochronous or split transfers) */
                    ehci_qh(c, link & ~0x1f);
                link = rd32(link & ~0x1f);
            }
        }
        if (e->cmd & E_ASE) {
            uint32_t start = e->asyncbase & ~0x1f, qh = start;
            for (int i = 0; i < 64; i++) {
                ehci_qh(c, qh);
                qh = rd32(qh) & ~0x1f;
                if (qh == start)
                    break;
            }
        }
    }
    if (e->cmd & E_IAAD) {                              /* doorbell: the async list was walked since */
        e->cmd &= ~E_IAAD;
        e->sts |= E_IAA;
    }
    ehci_irq(c);
}

static void
ehci_set_configflag(card_t *c, uint32_t val)
{
    ehci_t *e = &c->ehci;
    if ((val & 1) == e->configflag)
        return;
    e->configflag = val & 1;
    for (int p = 0; p < NPORTS; p++) {                  /* set: EHCI takes every port, clear: the companions do */
        e->portsc[p] = (e->portsc[p] & ~EP_PO) | (e->configflag ? 0 : EP_PO);
        port_route(c, p);
    }
}

static void
ehci_reset(card_t *c)
{
    ehci_t *e  = &c->ehci;
    e->cmd     = 0x80000;                               /* interrupt threshold 8 microframes */
    e->sts     = 0;                                     /* HCHalted is computed from RS */
    e->intr    = e->frindex = e->plbase = e->asyncbase = 0;
    e->configflag = 1;
    ehci_set_configflag(c, 0);
    dma_check(c);
    ehci_irq(c);
}

static void
ehci_port_write(card_t *c, int p, uint32_t val)
{
    uint32_t *sc = &c->ehci.portsc[p];
    usbdev_t *d  = &Dev[p];

    *sc &= ~(val & (EP_CSC | EP_PEC | EP_OCC));         /* write 1 to clear */
    if (!(val & EP_PE))                                 /* software can only disable */
        *sc &= ~EP_PE;
    if ((val & EP_PR) && !(*sc & EP_PR)) {
        *sc = (*sc & ~EP_PE) | EP_PR;
        d->addr = d->config = d->high_speed = 0;
    } else if (!(val & EP_PR) && (*sc & EP_PR)) {      /* reset done: a high-speed device is enabled */
        *sc &= ~EP_PR;
        if (d->present && ehci_owns(c, p) && !d->low_speed) {
            *sc |= EP_PE;
            d->high_speed = 1;
        }
    }
    if (c->ehci.configflag && ((val ^ *sc) & EP_PO)) {
        *sc = (*sc & ~EP_PO) | (val & EP_PO);
        port_route(c, p);
    }
}

static uint32_t
ehci_readl(uint32_t addr, void *priv)
{
    card_t *c = (card_t *) priv;
    ehci_t *e = &c->ehci;
    int     r = addr & 0xfc;

    switch (r) {
        case 0x00: return 0x01000010;                   /* HCIVERSION 1.00, CAPLENGTH 10h */
        case 0x04: return 0x00002204;                   /* 2 companions of 2 ports, 4 ports, no port power control */
        case 0x08: return 0x00006872;                   /* EECP 68h, programmable frame list size */
        case 0x10: return e->cmd;
        case 0x14: return e->sts | (e->cmd & E_RS ? 0 : E_HALTED) | (e->cmd & E_PSE ? E_PSS : 0)
                          | (e->cmd & E_ASE ? E_ASS : 0);
        case 0x18: return e->intr;
        case 0x1c: return e->frindex;
        case 0x24: return e->plbase;
        case 0x28: return e->asyncbase;
        case 0x50: return e->configflag;
        case 0x54 ... 0x60: {
            int      p = (r - 0x54) / 4;
            uint32_t v = e->portsc[p] | EP_PP;
            if (Dev[p].present && ehci_owns(c, p))
                v |= EP_CCS;
            if (Dev[p].present && !(v & EP_PE))
                v |= Dev[p].low_speed ? EP_LS_K : EP_LS_J;
            return v;
        }
    }
    return 0;
}

static void
ehci_writel(uint32_t addr, uint32_t val, void *priv)
{
    card_t *c = (card_t *) priv;
    ehci_t *e = &c->ehci;
    int     r = addr & 0xfc;

    switch (r) {
        case 0x10:
            if (val & E_HCRESET) {
                ehci_reset(c);
                return;
            }
            e->cmd = val & 0xff0b7f;
            dma_check(c);
            break;
        case 0x14: e->sts &= ~(val & 0x3f); break;
        case 0x18: e->intr = val & 0x3f; break;
        case 0x1c:
            if (!(e->cmd & E_RS))
                e->frindex = val & 0x3fff;
            break;
        case 0x24: e->plbase = val & ~0xfff; break;
        case 0x28: e->asyncbase = val & ~0x1f; break;
        case 0x50: ehci_set_configflag(c, val); break;
        case 0x54 ... 0x60: ehci_port_write(c, (r - 0x54) / 4, val); break;
    }
    ehci_irq(c);
}

static uint8_t
ehci_readb(uint32_t addr, void *priv)
{
    return ehci_readl(addr, priv) >> (8 * (addr & 3));
}

static uint16_t
ehci_readw(uint32_t addr, void *priv)
{
    return ehci_readl(addr, priv) >> (8 * (addr & 2));
}

static void
ehci_writeb(uint32_t addr, uint8_t val, void *priv)
{
    int sh = 8 * (addr & 3);
    ehci_writel(addr, (ehci_readl(addr, priv) & ~(0xffu << sh)) | (uint32_t) val << sh, priv);
}

static void
ehci_writew(uint32_t addr, uint16_t val, void *priv)
{
    int sh = 8 * (addr & 2);
    ehci_writel(addr, (ehci_readl(addr, priv) & ~(0xffffu << sh)) | (uint32_t) val << sh, priv);
}

static void
ehci_mmio(card_t *c)
{
    ehci_t  *e    = &c->ehci;
    uint32_t base = (e->pci[0x10] | e->pci[0x11] << 8 | e->pci[0x12] << 16 | (uint32_t) e->pci[0x13] << 24) & ~0xff;
    if (base && (e->pci[0x04] & 2))
        mem_mapping_set_addr(&e->mmio, base, 0x100);
    else
        mem_mapping_disable(&e->mmio);
}

static void
ehci_pci_write(card_t *c, int addr, uint8_t val)
{
    ehci_t *e = &c->ehci;
    switch (addr) {
        case 0x04: e->pci[0x04] = val & 0x06; ehci_mmio(c); dma_check(c); break;
        case 0x0d: e->pci[0x0d] = val; break;
        case 0x10: break;                               /* 256 bytes */
        case 0x11 ... 0x13: e->pci[addr] = val; ehci_mmio(c); break;
        case 0x3c: e->pci[0x3c] = val; break;
        case 0x40 ... 0x67:
        case 0x6c ... 0x6f: e->pci[addr] = val; break;
        case 0x6a: e->pci[addr] = val & 1; break;       /* USBLEGSUP: BIOS owned */
        case 0x6b: e->pci[addr] = val & 1; break;       /* OS owned */
    }
}

static void
ehci_init_pci(ehci_t *e)
{
    e->pci[0x00] = 0x06, e->pci[0x01] = 0x11, e->pci[0x02] = 0x04, e->pci[0x03] = 0x31;
    e->pci[0x06] = 0x10, e->pci[0x07] = 0x02, e->pci[0x08] = 0x63;
    e->pci[0x09] = 0x20, e->pci[0x0a] = 0x03, e->pci[0x0b] = 0x0c, e->pci[0x0e] = 0x80;
    e->pci[0x3d] = PCI_INTC, e->pci[0x60] = 0x20, e->pci[0x61] = 0x20, e->pci[0x62] = 0x01;
    e->pci[0x68] = 0x01;                                /* USB legacy support capability, nobody owns it */
}

/****************************************************************
 * The card
 ****************************************************************/

static void
card_frame(void *priv)
{
    card_t *c = (card_t *) priv;

    timer_on_auto(&c->frame_timer, 1000.0);
    Frames++;
    if (GoneAt && Frames >= GoneAt && Dev[1].present) {    /* disk-gone: unplugged */
        Dev[1].present = 0;
        port_route(c, 1);
        c->uhci[0].portsc[1] |= PORT_PEC;
    }
    for (int f = 0; f < 2; f++)
        uhci_frame(c, &c->uhci[f]);
    ehci_frame(c);
}

static uint8_t
card_pci_read(int func, int addr, int len, void *priv)
{
    const card_t *c = (const card_t *) priv;
    return func < 2 ? c->uhci[func].pci[addr & 0xff] : func == 2 ? c->ehci.pci[addr & 0xff] : 0xff;
}

static void
card_pci_write(int func, int addr, int len, uint8_t val, void *priv)
{
    card_t *c = (card_t *) priv;
    if (func < 2)
        uhci_pci_write(c, &c->uhci[func], addr & 0xff, val);
    else if (func == 2)
        ehci_pci_write(c, addr & 0xff, val);
}

static void
card_reset(card_t *c)
{
    ehci_reset(c);
    for (int f = 0; f < 2; f++)
        uhci_reset(c, &c->uhci[f]);
}

static void *
card_init(const device_t *info)
{
    card_t     *c    = calloc(1, sizeof(card_t));
    const char *disk = getenv("BOX86_USB_DISK"), *fault = getenv("BOX86_USB_FAULT");
    static const char *const faults[] = { "", "kbd-stall", "disk-nak", "disk-stall", "disk-csw", "disk-gone" };

    for (int f = 1; fault && f < 6; f++)
        if (!strcmp(fault, faults[f]))
            Fault = f;
    GoneAt = Frames = 0;
    memset(Dev, 0, sizeof(Dev));
    Card = c;

    for (int f = 0; f < 2; f++)
        uhci_init_pci(&c->uhci[f], f);
    ehci_init_pci(&c->ehci);
    if (getenv("BOX86_USB_KBD")) {
        usbdev_t *k = &Dev[0];
        k->present = 1, k->low_speed = 1, k->dev_desc = kbd_dev_desc, k->cfg_desc = kbd_cfg_desc, k->cfg_len = sizeof(kbd_cfg_desc);
        k->class_request = kbd_class, k->in = kbd_in, k->out = no_out;
    }
    if (disk && (Msc.img = fopen(disk, "r+b"))) {
        usbdev_t *m = &Dev[1];
        fseek(Msc.img, 0, SEEK_END);
        Msc.blocks = ftell(Msc.img) / 512;
        m->present = 1, m->dev_desc = msc_dev_desc, m->cfg_desc = msc_cfg_desc, m->hs_cfg_desc = msc_hs_cfg_desc;
        m->cfg_len = sizeof(msc_cfg_desc), m->class_request = msc_class, m->in = msc_in, m->out = msc_out;
    }
    mem_mapping_add(&c->ehci.mmio, 0, 0, ehci_readb, ehci_readw, ehci_readl, ehci_writeb, ehci_writew, ehci_writel,
                    NULL, MEM_MAPPING_EXTERNAL, c);
    mem_mapping_disable(&c->ehci.mmio);
    card_reset(c);
    pci_add_card(PCI_ADD_NORMAL, card_pci_read, card_pci_write, c, &c->slot);
    timer_add(&c->frame_timer, card_frame, c, 0);
    timer_on_auto(&c->frame_timer, 1000.0);
    return c;
}

static void
card_close(void *priv)
{
    if (Msc.img)
        fclose(Msc.img);
    free(Msc.data);
    memset(&Msc, 0, sizeof(Msc));
    Card = NULL;
    free(priv);
}

static void
card_device_reset(void *priv)
{
    card_t *c = (card_t *) priv;
    for (int f = 0; f < 2; f++) {
        c->uhci[f].pci[0x04] = 0;
        uhci_io(c, &c->uhci[f], 0);
    }
    c->ehci.pci[0x04] = 0;
    ehci_mmio(c);
    card_reset(c);
}

const device_t usb_uhci_via_device = {
    .name          = "VIA VT6212 USB (2x UHCI + EHCI) for tests",
    .internal_name = "usb_uhci_via",
    .flags         = DEVICE_PCI,
    .local         = 0,
    .init          = card_init,
    .close         = card_close,
    .reset         = card_device_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

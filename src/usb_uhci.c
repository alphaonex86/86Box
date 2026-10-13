/*
 * UHCI USB controller for firmware tests: a VIA VT83C572 (1106:3038 rev 61, the MB-8433UUD-A's USB card) whose
 * frame list, queue heads and transfer descriptors are walked from RAM by bus mastering every ms, as the hardware
 * does (nothing runs while PCI bus mastering is off). Root port 1: a HID boot keyboard (BOX86_USB_KBD=1; keys typed
 * by the test control's "usbkey"); port 2: a bulk-only mass storage disk on an image file (BOX86_USB_DISK=path).
 * The keyboard is low speed, as real ones: its TDs need the LS bit (else they time out), its port shows LSDA;
 * the disk is full speed. BOX86_USB_FAULT injects one fault: kbd-stall (the keyboard stalls its control requests),
 * disk-nak (the disk never sends data), disk-stall (it stalls the data of READ), disk-csw (bad CSW signature),
 * disk-gone (it is unplugged 3 s after its first READ). Enough of USB for a BIOS and an OS driver: control, interrupt and bulk transfers, SCSI
 * INQUIRY, TEST UNIT READY, REQUEST SENSE, READ CAPACITY, READ/WRITE(10), MODE SENSE.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/dma.h>
#include <86box/io.h>
#include <86box/pci.h>
#include <86box/timer.h>
#include <86box/usb_uhci.h>

enum { R_NAK = -2, R_STALL = -1 };
enum { F_NONE, F_KBD_STALL, F_DISK_NAK, F_DISK_STALL, F_DISK_CSW, F_DISK_GONE };
static int Fault;
static uint64_t GoneAt;                                /* disk-gone: the frame count it disappears at */
static uint64_t Frames;

typedef struct usbdev_t usbdev_t;
struct usbdev_t {
    int present, low_speed, addr, new_addr, config;
    const uint8_t *dev_desc, *cfg_desc;
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
                    memcpy(d->ctl, d->cfg_desc, n = d->cfg_len);
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
 * The controller
 ****************************************************************/

enum { CMD_RS = 1, CMD_HCRESET = 2, CMD_GRESET = 4 };
enum { STS_USBINT = 1, STS_ERROR = 2, STS_HALTED = 0x20 };
enum { PORT_CCS = 1, PORT_CSC = 2, PORT_PE = 4, PORT_PEC = 8, PORT_LSDA = 0x100, PORT_RESET = 0x200 };
enum { TD_ACTIVE = 1 << 23, TD_STALLED = 1 << 22, TD_NAK = 1 << 19, TD_TIMEOUT = 1 << 18, TD_IOC = 1 << 24,
       TD_LS = 1 << 26 };

typedef struct uhci_t {
    uint8_t    pci[256];
    uint8_t    slot, irq_state;
    uint16_t   io_base;
    uint16_t   cmd, sts, intr, frnum;
    uint32_t   flbase;
    uint8_t    sof;
    uint16_t   portsc[2];
    usbdev_t   dev[2];
    pc_timer_t frame_timer;
} uhci_t;

static uhci_t *Uhci;

static void
uhci_irq(uhci_t *u)
{
    if ((u->sts & (STS_USBINT | STS_ERROR)) && (u->intr & 0x0d))
        pci_set_irq(u->slot, PCI_INTA, &u->irq_state);
    else
        pci_clear_irq(u->slot, PCI_INTA, &u->irq_state);
}

static usbdev_t *
uhci_find(uhci_t *u, int addr)
{
    for (int p = 0; p < 2; p++)
        if (u->dev[p].present && (u->portsc[p] & PORT_PE) && !(u->portsc[p] & PORT_RESET) && u->dev[p].addr == addr)
            return &u->dev[p];
    return NULL;
}

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

/* One TD: 1 done (or not active), 0 NAK (left active), -1 error (stalled or no device). */
static int
uhci_td(uhci_t *u, uint32_t td, uint32_t *link)
{
    uint32_t status = rd32(td + 4), token = rd32(td + 8), buffer = rd32(td + 12);
    *link = rd32(td);
    if (!(status & TD_ACTIVE))
        return 1;
    int       pid = token & 0xff, addr = (token >> 8) & 0x7f, ep = (token >> 15) & 0xf;
    int       maxlen = ((token >> 21) + 1) & 0x7ff;
    uint8_t   buf[1024];
    usbdev_t *d = uhci_find(u, addr);
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
    return 1;
}

static void
uhci_frame(void *priv)
{
    uhci_t  *u    = (uhci_t *) priv;
    uint32_t link = rd32(u->flbase + (u->frnum & 0x3ff) * 4), next;

    timer_on_auto(&u->frame_timer, 1000.0);
    Frames++;
    if (GoneAt && Frames >= GoneAt && u->dev[1].present) {   /* disk-gone: unplugged */
        u->dev[1].present = 0;
        u->portsc[1]      = PORT_CSC | PORT_PEC;
    }
    if (!(u->cmd & CMD_RS) || !(u->pci[0x04] & 4))     /* stopped, or no bus mastering */
        return;
    for (int i = 0; i < 256 && !(link & 1); i++) {
        uint32_t addr = link & ~0xf;
        if (link & 2) {                                 /* queue head: its TDs until one is not done */
            uint32_t elem = rd32(addr + 4);
            for (int j = 0; j < 64 && !(elem & 3); j++) {
                if (uhci_td(u, elem & ~0xf, &next) <= 0)
                    break;
                elem = next;
                wr32(addr + 4, elem);
                if (!(next & 4))                        /* breadth first: one TD per queue per pass */
                    break;
            }
            link = rd32(addr);
        } else {
            uhci_td(u, addr, &next);
            link = next;
        }
    }
    u->frnum = (u->frnum + 1) & 0x7ff;
    uhci_irq(u);
}

static void
uhci_port_reset(uhci_t *u)
{
    for (int p = 0; p < 2; p++) {
        u->portsc[p] = u->dev[p].present ? PORT_CCS | PORT_CSC | (u->dev[p].low_speed ? PORT_LSDA : 0) : 0;
        u->dev[p].addr = u->dev[p].config = 0;
    }
}

static void
uhci_reset(uhci_t *u)
{
    u->cmd = u->intr = u->frnum = 0;
    u->sts    = STS_HALTED;
    u->flbase = 0;
    u->sof    = 0x40;
    uhci_port_reset(u);
}

static uint16_t
uhci_readw(uint16_t port, void *priv)
{
    uhci_t *u = (uhci_t *) priv;
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
    uhci_t *u = (uhci_t *) priv;
    switch (port - u->io_base) {
        case 0x00:
            if (val & (CMD_HCRESET | CMD_GRESET)) {
                uhci_reset(u);
                break;
            }
            u->cmd = val;
            u->sts = (u->sts & ~STS_HALTED) | (val & CMD_RS ? 0 : STS_HALTED);
            break;
        case 0x02: u->sts &= ~(val & 0x1f); uhci_irq(u); break;
        case 0x04: u->intr = val & 0x0f; uhci_irq(u); break;
        case 0x06: u->frnum = val & 0x7ff; break;
        case 0x08: u->flbase = (u->flbase & 0xffff0000) | (val & 0xf000); break;
        case 0x0a: u->flbase = (u->flbase & 0xffff) | (uint32_t) val << 16; break;
        case 0x0c: u->sof = val & 0x7f; break;
        case 0x10:
        case 0x12: {
            int       p  = (port - u->io_base - 0x10) / 2;
            uint16_t *sc = &u->portsc[p];
            if (val & PORT_RESET) {
                u->dev[p].addr = u->dev[p].config = 0;
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
uhci_io(uhci_t *u, int on)
{
    if (u->io_base)
        io_removehandler(u->io_base, 0x20, uhci_readb, uhci_readw, uhci_readl, uhci_writeb, uhci_writew, uhci_writel, u);
    u->io_base = (u->pci[0x20] | u->pci[0x21] << 8) & 0xffe0;
    if (on && u->io_base && (u->pci[0x04] & 1))
        io_sethandler(u->io_base, 0x20, uhci_readb, uhci_readw, uhci_readl, uhci_writeb, uhci_writew, uhci_writel, u);
    else
        u->io_base = 0;
}

static uint8_t
uhci_pci_read(int func, int addr, int len, void *priv)
{
    const uhci_t *u = (const uhci_t *) priv;
    return func ? 0xff : u->pci[addr & 0xff];
}

static void
uhci_pci_write(int func, int addr, int len, uint8_t val, void *priv)
{
    uhci_t *u = (uhci_t *) priv;
    if (func)
        return;
    switch (addr) {
        case 0x04: u->pci[0x04] = val & 0x07; uhci_io(u, 1); break;
        case 0x0d: u->pci[0x0d] = val; break;
        case 0x20: u->pci[0x20] = (val & 0xe0) | 1; uhci_io(u, 1); break;
        case 0x21: u->pci[0x21] = val; uhci_io(u, 1); break;
        case 0x3c: u->pci[0x3c] = val; break;
        case 0x40 ... 0x4f:
        case 0xc0 ... 0xc1: u->pci[addr] = val; break;
    }
}

static void *
uhci_init(const device_t *info)
{
    uhci_t     *u    = calloc(1, sizeof(uhci_t));
    const char *disk = getenv("BOX86_USB_DISK"), *fault = getenv("BOX86_USB_FAULT");
    static const char *const faults[] = { "", "kbd-stall", "disk-nak", "disk-stall", "disk-csw", "disk-gone" };

    for (int f = 1; fault && f < 6; f++)
        if (!strcmp(fault, faults[f]))
            Fault = f;
    GoneAt = Frames = 0;

    u->pci[0x00] = 0x06, u->pci[0x01] = 0x11, u->pci[0x02] = 0x38, u->pci[0x03] = 0x30;
    u->pci[0x06] = 0x10, u->pci[0x07] = 0x02, u->pci[0x08] = 0x61;
    u->pci[0x09] = 0x00, u->pci[0x0a] = 0x03, u->pci[0x0b] = 0x0c;
    u->pci[0x20] = 0x01, u->pci[0x3d] = PCI_INTA, u->pci[0x60] = 0x10, u->pci[0xc1] = 0x20;
    if (getenv("BOX86_USB_KBD")) {
        usbdev_t *k = &u->dev[0];
        k->present = 1, k->low_speed = 1, k->dev_desc = kbd_dev_desc, k->cfg_desc = kbd_cfg_desc, k->cfg_len = sizeof(kbd_cfg_desc);
        k->class_request = kbd_class, k->in = kbd_in, k->out = no_out;
    }
    if (disk && (Msc.img = fopen(disk, "r+b"))) {
        usbdev_t *m = &u->dev[1];
        fseek(Msc.img, 0, SEEK_END);
        Msc.blocks = ftell(Msc.img) / 512;
        m->present = 1, m->dev_desc = msc_dev_desc, m->cfg_desc = msc_cfg_desc, m->cfg_len = sizeof(msc_cfg_desc);
        m->class_request = msc_class, m->in = msc_in, m->out = msc_out;
    }
    uhci_reset(u);
    pci_add_card(PCI_ADD_NORMAL, uhci_pci_read, uhci_pci_write, u, &u->slot);
    timer_add(&u->frame_timer, uhci_frame, u, 0);
    timer_on_auto(&u->frame_timer, 1000.0);
    Uhci = u;
    return u;
}

static void
uhci_close(void *priv)
{
    uhci_t *u = (uhci_t *) priv;
    if (Msc.img)
        fclose(Msc.img);
    free(Msc.data);
    memset(&Msc, 0, sizeof(Msc));
    Uhci = NULL;
    free(u);
}

static void
uhci_device_reset(void *priv)
{
    uhci_t *u = (uhci_t *) priv;
    uhci_reset(u);
    u->pci[0x04] = 0;
    uhci_io(u, 0);
}

const device_t usb_uhci_via_device = {
    .name          = "VIA VT83C572 USB (UHCI) for tests",
    .internal_name = "usb_uhci_via",
    .flags         = DEVICE_PCI,
    .local         = 0,
    .init          = uhci_init,
    .close         = uhci_close,
    .reset         = uhci_device_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

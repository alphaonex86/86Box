/* UHCI controller with a USB keyboard and disk for firmware tests (usb_uhci.c). */
#ifndef EMU_USB_UHCI_H
#define EMU_USB_UHCI_H
extern const device_t usb_uhci_via_device;
extern void usb_uhci_key(int mods_usage);   /* modifiers << 8 | HID usage: pressed, then released */
#endif

#ifndef __usbconfighw_h_included__
#define __usbconfighw_h_included__

/* Hardware specicic overrides for usbconfig.h */

#define USB_CFG_IOPORTNAME              D
#define USB_CFG_DMINUS_BIT              3
#define USB_CFG_DPLUS_BIT               2

#define USB_CFG_DEVICE_ID               0x16, 0x00

/* This is the ID of the device, low byte first. It is interpreted in the
 * scope of the vendor ID. The only requirement is that no two devices may
 * share the same product and vendor IDs. Not even if the devices are never
 * on the same bus together!
 */


#endif

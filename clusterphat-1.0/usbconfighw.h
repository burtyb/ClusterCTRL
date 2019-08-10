#ifndef __usbconfighw_h_included__
#define __usbconfighw_h_included__

/* Hardware specicic overrides for usbconfig.h */

#define USB_CFG_IOPORTNAME              D
#define USB_CFG_DMINUS_BIT              4
#define USB_CFG_DPLUS_BIT               3
#define USB_INTR_CFG_SET                ((1 << ISC11) | (1 << ISC10))
#define USB_INTR_ENABLE_BIT             INT1
#define USB_INTR_PENDING_BIT            INTF1
#define USB_INTR_VECTOR                 INT1_vect

#define USB_CFG_DEVICE_ID               0x12, 0x00
/* This is the ID of the device, low byte first. It is interpreted in the
 * scope of the vendor ID. The only requirement is that no two devices may
 * share the same product and vendor IDs. Not even if the devices are never
 * on the same bus together!
 */

#endif

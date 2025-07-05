/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usb_ch32_usbhs_reg.h"

#ifndef USB_NUM_BIDIR_ENDPOINTS
#define USB_NUM_BIDIR_ENDPOINTS 16
#endif

#define USB_SET_RX_DMA(ep_idx, addr) (*(volatile uint32_t *)((uint32_t)(&USBHS_DEVICE->UEP1_RX_DMA) + 4 * (ep_idx - 1)) = addr)
#define USB_SET_TX_DMA(ep_idx, addr) (*(volatile uint32_t *)((uint32_t)(&USBHS_DEVICE->UEP1_TX_DMA) + 4 * (ep_idx - 1)) = addr)
#define USB_SET_MAX_LEN(ep_idx, len) (*(volatile uint16_t *)((uint32_t)(&USBHS_DEVICE->UEP0_MAX_LEN) + 4 * ep_idx) = len)
#define USB_SET_TX_LEN(ep_idx, len)  (*(volatile uint16_t *)((uint32_t)(&USBHS_DEVICE->UEP0_TX_LEN) + 4 * ep_idx) = len)
#define USB_GET_TX_LEN(ep_idx)       (*(volatile uint16_t *)((uint32_t)(&USBHS_DEVICE->UEP0_TX_LEN) + 4 * ep_idx))
#define USB_SET_TX_CTRL(ep_idx, val) (*(volatile uint8_t *)((uint32_t)(&USBHS_DEVICE->UEP0_TX_CTRL) + 4 * ep_idx) = val)
#define USB_GET_TX_CTRL(ep_idx)      (*(volatile uint8_t *)((uint32_t)(&USBHS_DEVICE->UEP0_TX_CTRL) + 4 * ep_idx))
#define USB_SET_RX_CTRL(ep_idx, val) (*(volatile uint8_t *)((uint32_t)(&USBHS_DEVICE->UEP0_RX_CTRL) + 4 * ep_idx) = val)
#define USB_GET_RX_CTRL(ep_idx)      (*(volatile uint8_t *)((uint32_t)(&USBHS_DEVICE->UEP0_RX_CTRL) + 4 * ep_idx))

#define USB_SET_TX_ACK(ep_idx) USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx) & ~USBHS_EP_T_RES_MASK) | USBHS_EP_T_RES_ACK);
#define USB_SET_RX_ACK(ep_idx) USB_SET_RX_CTRL(ep_idx, (USB_GET_RX_CTRL(ep_idx) & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_ACK);

#define USB_SET_TX_NAK(ep_idx) USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx) & ~USBHS_EP_T_RES_MASK) | USBHS_EP_T_RES_NAK);
#define USB_SET_RX_NAK(ep_idx) USB_SET_RX_CTRL(ep_idx, (USB_GET_RX_CTRL(ep_idx) & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_NAK);

#define USB_SET_TX_TOG(ep_idx, val) USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx) & ~USBHS_EP_T_TOG_MASK) | val);
#define USB_SET_RX_TOG(ep_idx, val) USB_SET_RX_CTRL(ep_idx, (USB_GET_RX_CTRL(ep_idx) & ~USBHS_EP_R_TOG_MASK) | val);

/* Endpoint state */
struct ch32_usbhs_ep_state {
    uint16_t ep_mps;    /* Endpoint max packet size */
    uint8_t ep_type;    /* Endpoint type */
    uint8_t ep_stalled; /* Endpoint stall flag */
    uint8_t ep_enable;  /* Endpoint enable */
    uint8_t *xfer_buf;
    uint32_t xfer_len;
    uint32_t actual_xfer_len;
};

/* Driver state */
struct ch32_usbhs_udc {
    __attribute__((aligned(4))) struct usb_setup_packet setup;
    volatile uint8_t dev_addr;
    struct ch32_usbhs_ep_state in_ep[USB_NUM_BIDIR_ENDPOINTS];  /*!< IN endpoint parameters*/
    struct ch32_usbhs_ep_state out_ep[USB_NUM_BIDIR_ENDPOINTS]; /*!< OUT endpoint parameters */
} g_ch32_usbhs_udc[CONFIG_USBDEV_MAX_BUS];

__WEAK void usb_dc_low_level_init(void)
{
}

__WEAK void usb_dc_low_level_deinit(void)
{
}

int usb_dc_init(uint8_t busid)
{
    usb_dc_low_level_init();

    USBHS_DEVICE->HOST_CTRL = 0x00;
    USBHS_DEVICE->HOST_CTRL = USBHS_PHY_SUSPENDM;

    USBHS_DEVICE->CONTROL = 0;
#ifdef CONFIG_USB_HS
    USBHS_DEVICE->CONTROL = USBHS_DMA_EN | USBHS_INT_BUSY_EN | USBHS_HIGH_SPEED;
#else
    USBHS_DEVICE->CONTROL = USBHS_DMA_EN | USBHS_INT_BUSY_EN | USBHS_FULL_SPEED;
#endif

    USBHS_DEVICE->INT_FG = 0xff;
    USBHS_DEVICE->INT_EN = 0;
    USBHS_DEVICE->INT_EN = USBHS_SETUP_ACT_EN | USBHS_TRANSFER_EN | USBHS_BUS_RST_EN;

    USBHS_DEVICE->ENDP_TYPE = 0x00;
    USBHS_DEVICE->BUF_MODE = 0x00;

    USBHS_DEVICE->CONTROL |= USBHS_DEV_PU_EN;

    return 0;
}

int usb_dc_deinit(uint8_t busid)
{
    return 0;
}

int usbd_set_address(uint8_t busid, const uint8_t addr)
{
    if (addr == 0) {
        USBHS_DEVICE->DEV_AD = addr & 0xff;
    }
    g_ch32_usbhs_udc[busid].dev_addr = addr;
    return 0;
}

int usbd_set_remote_wakeup(uint8_t busid)
{
    return -1;
}

uint8_t usbd_get_port_speed(uint8_t busid)
{
    switch (USBHS_DEVICE->CONTROL & USBHS_SPEED_MASK) {
        case USBHS_FULL_SPEED:
            return USB_SPEED_FULL;
            break;
        case USBHS_HIGH_SPEED:
            return USB_SPEED_HIGH;
            break;
        case USBHS_LOW_SPEED:
            return USB_SPEED_LOW;
            break;
        default:
            return USB_SPEED_UNKNOWN;
            break;
    }

    return USB_SPEED_UNKNOWN;
}

int usbd_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep->bEndpointAddress);

    if (USB_EP_DIR_IS_OUT(ep->bEndpointAddress)) {
        g_ch32_usbhs_udc[busid].out_ep[ep_idx].ep_mps = USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize);
        g_ch32_usbhs_udc[busid].out_ep[ep_idx].ep_type = USB_GET_ENDPOINT_TYPE(ep->bmAttributes);
        g_ch32_usbhs_udc[busid].out_ep[ep_idx].ep_enable = true;
        if (g_ch32_usbhs_udc[busid].out_ep[ep_idx].ep_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            USBHS_DEVICE->ENDP_TYPE |= (1 << (ep_idx + 16));
        } else {
            USBHS_DEVICE->ENDP_TYPE &= ~(1 << (ep_idx + 16));
        }
        USBHS_DEVICE->ENDP_CONFIG |= (1 << (ep_idx + 16));
        USB_SET_RX_CTRL(ep_idx, USBHS_EP_R_RES_NAK | USBHS_EP_R_TOG_0 | USBHS_EP_R_AUTOTOG);
    } else {
        g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps = USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize);
        g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_type = USB_GET_ENDPOINT_TYPE(ep->bmAttributes);
        g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_enable = true;
        if (g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            USBHS_DEVICE->ENDP_TYPE |= (1 << (ep_idx));
        } else {
            USBHS_DEVICE->ENDP_TYPE &= ~(1 << (ep_idx));
        }
        USB_SET_TX_CTRL(ep_idx, USBHS_EP_T_RES_NAK | USBHS_EP_T_TOG_0 | USBHS_EP_T_AUTOTOG);
        USBHS_DEVICE->ENDP_CONFIG |= (1 << (ep_idx));
    }
    USB_SET_MAX_LEN(ep_idx, USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize));
    return 0;
}

int usbd_ep_close(uint8_t busid, const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    if (USB_EP_DIR_IS_OUT(ep)) {
        USBHS_DEVICE->ENDP_CONFIG &= ~(1 << (ep_idx + 16));
    } else {
        USBHS_DEVICE->ENDP_CONFIG &= ~(1 << (ep_idx));
    }
    return 0;
}

int usbd_ep_set_stall(uint8_t busid, const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (USB_EP_DIR_IS_OUT(ep)) {
        if (ep_idx == 0) {
            USBHS_DEVICE->UEP0_RX_CTRL = USBHS_EP_R_RES_STALL;
        } else {
            USB_SET_RX_CTRL(ep_idx, (USB_GET_RX_CTRL(ep_idx) & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_STALL);
        }
    } else {
        if (ep_idx == 0) {
            USBHS_DEVICE->UEP0_TX_CTRL = USBHS_EP_T_RES_STALL;
        } else {
            USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx) & ~USBHS_EP_T_RES_MASK) | USBHS_EP_T_RES_STALL);
        }
    }

    return 0;
}

int usbd_ep_clear_stall(uint8_t busid, const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (USB_EP_DIR_IS_OUT(ep)) {
        USB_SET_RX_CTRL(ep_idx, USBHS_EP_R_RES_ACK | USBHS_EP_R_TOG_0);
    } else {
        USB_SET_TX_CTRL(ep_idx, USBHS_EP_T_RES_NAK | USBHS_EP_T_TOG_0);
    }
    return 0;
}

int usbd_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (USB_EP_DIR_IS_OUT(ep)) {
        *stalled = USB_GET_RX_CTRL(ep_idx) & USBHS_EP_R_RES_STALL ? 1 : 0;
    } else {
        *stalled = USB_GET_TX_CTRL(ep_idx) & USBHS_EP_T_RES_STALL ? 1 : 0;
    }
    return 0;
}

int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (!data && data_len) {
        return -1;
    }
    if (!g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_enable) {
        return -2;
    }
    if ((uint32_t)data & 0x03) {
        return -3;
    }

    g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_buf = (uint8_t *)data;
    g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len = data_len;
    g_ch32_usbhs_udc[busid].in_ep[ep_idx].actual_xfer_len = 0;

    if (data_len == 0) {
        USB_SET_TX_LEN(ep_idx, 0);
    } else {
        data_len = MIN(data_len, g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps);
        USB_SET_TX_LEN(ep_idx, data_len);
        if (ep_idx == 0) {
            USBHS_DEVICE->UEP0_DMA = (uint32_t)data;
        } else {
            USB_SET_TX_DMA(ep_idx, (uint32_t)data);
        }
    }
    USB_SET_TX_ACK(ep_idx);
    return 0;
}

int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (!data && data_len) {
        return -1;
    }
    if (!g_ch32_usbhs_udc[busid].out_ep[ep_idx].ep_enable) {
        return -2;
    }
    if ((uint32_t)data & 0x03) {
        return -3;
    }

    g_ch32_usbhs_udc[busid].out_ep[ep_idx].xfer_buf = (uint8_t *)data;
    g_ch32_usbhs_udc[busid].out_ep[ep_idx].xfer_len = data_len;
    g_ch32_usbhs_udc[busid].out_ep[ep_idx].actual_xfer_len = 0;

    if (data_len != 0) {
        if (ep_idx == 0) {
            USBHS_DEVICE->UEP0_DMA = (uint32_t)data;
        } else {
            USB_SET_RX_DMA(ep_idx, (uint32_t)data);
        }
    }
    USB_SET_RX_ACK(ep_idx);

    return 0;
}

void USBD_IRQHandler(uint8_t busid)
{
    uint8_t intflag = 0;

    intflag = USBHS_DEVICE->INT_FG;

    if (intflag & USBHS_TRANSFER_FLAG) {
        uint32_t ep_idx, token, write_count, read_count;
        ep_idx = (USBHS_DEVICE->INT_ST) & MASK_UIS_ENDP;
        token = (((USBHS_DEVICE->INT_ST) & MASK_UIS_TOKEN) >> 4) & 0x03;

        switch (token) {
            case PID_IN:
                USB_SET_TX_NAK(ep_idx);

                if (ep_idx == 0x00) {
                    USB_GET_TX_CTRL(ep_idx) ^= USBHS_EP_T_TOG_1;
                    if (g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len >= g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps) {
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len -= g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps;
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].actual_xfer_len += g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps;
                    } else {
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].actual_xfer_len += g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len;
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len = 0;
                        USB_SET_TX_TOG(ep_idx, USBHS_EP_T_TOG_1);
                    }

                    usbd_event_ep_in_complete_handler(busid, ep_idx | 0x80, g_ch32_usbhs_udc[busid].in_ep[ep_idx].actual_xfer_len);

                    if (g_ch32_usbhs_udc[busid].setup.bRequest == USB_REQUEST_SET_ADDRESS) {
                        USBHS_DEVICE->DEV_AD = g_ch32_usbhs_udc[busid].dev_addr & 0xff;
                    }

                    if (g_ch32_usbhs_udc[busid].setup.wLength && ((g_ch32_usbhs_udc[busid].setup.bmRequestType & USB_REQUEST_DIR_MASK) == USB_REQUEST_DIR_OUT)) {
                        /* In status, start reading setup */
                        USBHS_DEVICE->UEP0_DMA = (uint32_t)&g_ch32_usbhs_udc[busid].setup;
                        USB_SET_RX_ACK(ep_idx);
                    } else if (g_ch32_usbhs_udc[busid].setup.wLength == 0) {
                        /* In status, start reading setup */
                        USBHS_DEVICE->UEP0_DMA = (uint32_t)&g_ch32_usbhs_udc[busid].setup;
                        USB_SET_RX_ACK(ep_idx);
                    }
                } else {
                    if (g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len > g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps) {
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_buf += g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps;
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len -= g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps;
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].actual_xfer_len += g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps;

                        write_count = MIN(g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len, g_ch32_usbhs_udc[busid].in_ep[ep_idx].ep_mps);
                        USB_SET_TX_LEN(ep_idx, write_count);
                        USB_SET_TX_DMA(ep_idx, (uint32_t)g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_buf);

                        USB_SET_TX_ACK(ep_idx);
                    } else {
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].actual_xfer_len += g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len;
                        g_ch32_usbhs_udc[busid].in_ep[ep_idx].xfer_len = 0;

                        usbd_event_ep_in_complete_handler(busid, ep_idx | 0x80, g_ch32_usbhs_udc[busid].in_ep[ep_idx].actual_xfer_len);
                    }
                }
                break;
            case PID_OUT:
                read_count = USBHS_DEVICE->RX_LEN;
                if (USBHS_DEVICE->INT_ST & USBHS_DEV_UIS_TOG_OK) {
                    USB_SET_RX_NAK(ep_idx);

                    g_ch32_usbhs_udc[busid].out_ep[ep_idx].actual_xfer_len += read_count;
                    g_ch32_usbhs_udc[busid].out_ep[ep_idx].xfer_len -= read_count;
                    if (ep_idx == 0x00) {
                        USB_GET_RX_CTRL(ep_idx) ^= USBHS_EP_R_TOG_1;
                        usbd_event_ep_out_complete_handler(busid, 0x00, g_ch32_usbhs_udc[busid].out_ep[ep_idx].actual_xfer_len);

                        if (read_count == 0) {
                            /* Out status, start reading setup */
                            USBHS_DEVICE->UEP0_DMA = (uint32_t)&g_ch32_usbhs_udc[busid].setup;
                            USBHS_DEVICE->UEP0_RX_CTRL = USBHS_EP_R_RES_ACK | USBHS_EP_R_TOG_1;
                            USB_SET_TX_TOG(ep_idx, USBHS_EP_T_TOG_1);
                        }
                    } else {
                        g_ch32_usbhs_udc[busid].out_ep[ep_idx].xfer_buf += read_count;

                        if ((read_count < g_ch32_usbhs_udc[busid].out_ep[ep_idx].ep_mps) || (g_ch32_usbhs_udc[busid].out_ep[ep_idx].xfer_len == 0)) {
                            usbd_event_ep_out_complete_handler(busid, ep_idx, g_ch32_usbhs_udc[busid].out_ep[ep_idx].actual_xfer_len);
                        } else {
                            USB_SET_RX_DMA(ep_idx, (uint32_t)g_ch32_usbhs_udc[busid].out_ep[ep_idx].xfer_buf);
                            USB_SET_RX_ACK(ep_idx);
                        }
                    }
                }
                break;
            default:
                break;
        }

        USBHS_DEVICE->INT_FG = USBHS_TRANSFER_FLAG;
    }

    if (intflag & USBHS_SETUP_FLAG) {
        USB_SET_TX_CTRL(0, USBHS_EP_T_TOG_1 | USBHS_EP_T_RES_NAK);
        USB_SET_RX_CTRL(0, USBHS_EP_R_TOG_1 | USBHS_EP_R_RES_NAK);

        usbd_event_ep0_setup_complete_handler(busid, (uint8_t *)&g_ch32_usbhs_udc[busid].setup);

        USBHS_DEVICE->INT_FG = USBHS_SETUP_FLAG;
    }

    if (intflag & USBHS_BUS_RST_FLAG) {
        USBHS_DEVICE->ENDP_CONFIG = USBHS_EP0_T_EN | USBHS_EP0_R_EN;

        USBHS_DEVICE->UEP0_TX_LEN = 0;
        USBHS_DEVICE->UEP0_TX_CTRL = USBHS_EP_T_RES_NAK;

        for (uint8_t ep_idx = 0; ep_idx < USB_NUM_BIDIR_ENDPOINTS; ep_idx++) {
            USB_SET_TX_LEN(ep_idx, 0);
            USB_SET_TX_CTRL(ep_idx, USBHS_EP_T_AUTOTOG | USBHS_EP_T_TOG_0 | USBHS_EP_T_RES_NAK);
            USB_SET_RX_CTRL(ep_idx, USBHS_EP_R_AUTOTOG | USBHS_EP_R_TOG_0 | USBHS_EP_R_RES_NAK);
        }

        memset(&g_ch32_usbhs_udc[busid], 0, sizeof(struct ch32_usbhs_udc));
        usbd_event_reset_handler(busid);
        USBHS_DEVICE->UEP0_DMA = (uint32_t)&g_ch32_usbhs_udc[busid].setup;
        USBHS_DEVICE->UEP0_RX_CTRL = USBHS_EP_R_RES_ACK;

        USBHS_DEVICE->INT_FG = USBHS_BUS_RST_FLAG;
    }
}

void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBHS_IRQHandler(void)
{
    extern void USBD_IRQHandler(uint8_t busid);
    USBD_IRQHandler(0);
}

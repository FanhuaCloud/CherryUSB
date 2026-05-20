/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usb_ch58x_usbfs_reg.h"

/**
 * @brief   Related register macro
 */
#define USB0_BASE 0x40008000u
#define USB1_BASE 0x40008400u

#define CH58x_USBFS_DEV ((USB_FS_TypeDef *)g_usbdev_bus[busid].reg_base)

/*!< 8-bit value of endpoint control register */
#define EPn_CTRL(epid) \
    *(volatile uint8_t *)(&(CH58x_USBFS_DEV->UEP0_CTRL) + epid * 4 + (epid / 5) * 48)

/*!< The length register value of the endpoint send buffer */
#define EPn_TX_LEN(epid) \
    *(volatile uint8_t *)(&(CH58x_USBFS_DEV->UEP0_T_LEN) + epid * 4 + (epid / 5) * 48)

/*!< Read setup packet to use in ep0 in */
#define GET_SETUP_PACKET(data_add) \
    *(struct usb_setup_packet *)data_add

/*!< Set epid ep tx valid // Not an isochronous endpoint  */
#define EPn_SET_TX_VALID(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
/*!< Set epid ep rx valid // Not an isochronous endpoint */
#define EPn_SET_RX_VALID(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
/*!< Set epid ep tx valid // Isochronous endpoint */
#define EPn_SET_TX_ISO_VALID(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_T_RES) | UEP_T_RES_TOUT;
/*!< Set epid ep rx valid // Isochronous endpoint */
#define EPn_SET_RX_ISO_VALID(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_R_RES) | UEP_R_RES_TOUT;
/*!< Set epid ep tx nak */
#define EPn_SET_TX_NAK(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
/*!< Set epid ep rx nak */
#define EPn_SET_RX_NAK(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
/*!< Set epid ep tx stall */
#define EPn_SET_TX_STALL(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_T_RES) | UEP_T_RES_STALL
/*!< Set epid ep rx stall */
#define EPn_SET_RX_STALL(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~MASK_UEP_R_RES) | UEP_R_RES_STALL
/*!< Clear epid ep tx stall */
#define EPn_CLR_TX_STALL(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK
/*!< Clear epid ep rx stall */
#define EPn_CLR_RX_STALL(epid) \
    EPn_CTRL(epid) = (EPn_CTRL(epid) & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK
/*!< Set epid ep tx len */
#define EPn_SET_TX_LEN(epid, len) \
    EPn_TX_LEN(epid) = len
/*!< Get epid ep rx len */
#define EPn_GET_RX_LEN(epid) \
    CH58x_USBFS_DEV->USB_RX_LEN

/*!< ep nums */
#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 5
#endif
/*!< ep mps */
#define EP_MPS 64
/*!< set ep4 in mps 64 */
#define EP4_IN_MPS EP_MPS
/*!< set ep4 out mps 64 */
#define EP4_OUT_MPS EP_MPS

/*!< User defined assignment endpoint RAM */
static USB_MEM_ALIGNX uint8_t ep0_data_buff[CONFIG_USBDEV_MAX_BUS][64 + EP4_OUT_MPS + EP4_IN_MPS]; /*!< ep0(64)+ep4_out(64)+ep4_in(64) */
static USB_MEM_ALIGNX uint8_t ep1_data_buff[CONFIG_USBDEV_MAX_BUS][64 + 64];                       /*!< ep1_out(64)+ep1_in(64) */
static USB_MEM_ALIGNX uint8_t ep2_data_buff[CONFIG_USBDEV_MAX_BUS][64 + 64];                       /*!< ep2_out(64)+ep2_in(64) */
static USB_MEM_ALIGNX uint8_t ep3_data_buff[CONFIG_USBDEV_MAX_BUS][64 + 64];                       /*!< ep3_out(64)+ep3_in(64) */
#if (CONFIG_USBDEV_EP_NUM == 8)
/**
 * This dcd porting can be used on ch581, ch582, ch583,
 * and also on ch571, ch572, and ch573. Note that only five endpoints are available for ch571, ch572, and ch573.
 */
static USB_MEM_ALIGNX uint8_t ep5_data_buff[CONFIG_USBDEV_MAX_BUS][64 + 64]; /*!< ep5_out(64)+ep5_in(64) */
static USB_MEM_ALIGNX uint8_t ep6_data_buff[CONFIG_USBDEV_MAX_BUS][64 + 64]; /*!< ep6_out(64)+ep6_in(64) */
static USB_MEM_ALIGNX uint8_t ep7_data_buff[CONFIG_USBDEV_MAX_BUS][64 + 64]; /*!< ep7_out(64)+ep7_in(64) */
#endif
/**
 * @brief   Endpoint information structure
 */
typedef struct _usbd_ep_info {
    uint8_t mps;          /*!< Maximum packet length of endpoint */
    uint8_t eptype;       /*!< Endpoint Type */
    uint8_t *ep_ram_addr; /*!< Endpoint buffer address */

    uint8_t ep_enable; /* Endpoint enable */
    uint8_t *xfer_buf;
    uint32_t xfer_len;
    uint32_t actual_xfer_len;
} usbd_ep_info;

/*!< ch58x usb */
static struct _ch58x_core_prvi {
    uint8_t address; /*!< Address */
    usbd_ep_info ep_in[CONFIG_USBDEV_EP_NUM];
    usbd_ep_info ep_out[CONFIG_USBDEV_EP_NUM];
    struct usb_setup_packet setup;
} usb_dc_cfg[CONFIG_USBDEV_MAX_BUS];

__WEAK void usb_dc_low_level_init(uint8_t busid)
{
    (void)busid;
}

__WEAK void usb_dc_low_level_deinit(uint8_t busid)
{
    (void)busid;
}

/**
 * @brief            USB initialization
 * @pre              None
 * @param[in]        None
 * @retval           >=0 success otherwise failure
 */
int usb_dc_init(uint8_t busid)
{
    memset(&usb_dc_cfg[busid], 0, sizeof(usb_dc_cfg[busid]));

    usb_dc_cfg[busid].ep_in[0].ep_ram_addr = ep0_data_buff[busid];
    usb_dc_cfg[busid].ep_out[0].ep_ram_addr = ep0_data_buff[busid];

    usb_dc_cfg[busid].ep_in[1].ep_ram_addr = ep1_data_buff[busid] + 64;
    usb_dc_cfg[busid].ep_out[1].ep_ram_addr = ep1_data_buff[busid];

    usb_dc_cfg[busid].ep_in[2].ep_ram_addr = ep2_data_buff[busid] + 64;
    usb_dc_cfg[busid].ep_out[2].ep_ram_addr = ep2_data_buff[busid];

    usb_dc_cfg[busid].ep_in[3].ep_ram_addr = ep3_data_buff[busid] + 64;
    usb_dc_cfg[busid].ep_out[3].ep_ram_addr = ep3_data_buff[busid];

    usb_dc_cfg[busid].ep_in[4].ep_ram_addr = ep0_data_buff[busid] + 64 + EP4_OUT_MPS;
    usb_dc_cfg[busid].ep_out[4].ep_ram_addr = ep0_data_buff[busid] + 64;
#if (CONFIG_USBDEV_EP_NUM == 8)
    usb_dc_cfg[busid].ep_in[5].ep_ram_addr = ep5_data_buff[busid] + 64;
    usb_dc_cfg[busid].ep_out[5].ep_ram_addr = ep5_data_buff[busid];

    usb_dc_cfg[busid].ep_in[6].ep_ram_addr = ep6_data_buff[busid] + 64;
    usb_dc_cfg[busid].ep_out[6].ep_ram_addr = ep6_data_buff[busid];

    usb_dc_cfg[busid].ep_in[7].ep_ram_addr = ep7_data_buff[busid] + 64;
    usb_dc_cfg[busid].ep_out[7].ep_ram_addr = ep7_data_buff[busid];
#endif
    /*!< Set the mode first and cancel RB_UC_CLR_ALL */
    CH58x_USBFS_DEV->USB_CTRL = 0x00;
    CH58x_USBFS_DEV->UEP4_1_MOD = RB_UEP4_RX_EN | RB_UEP4_TX_EN | RB_UEP1_RX_EN | RB_UEP1_TX_EN; /*!< EP4 OUT+IN   EP1 OUT+IN */
    CH58x_USBFS_DEV->UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN; /*!< EP2 OUT+IN   EP3 OUT+IN */
#if (CONFIG_USBDEV_EP_NUM == 8)
    CH58x_USBFS_DEV->UEP567_MOD = RB_UEP5_RX_EN | RB_UEP5_TX_EN | RB_UEP6_RX_EN | RB_UEP6_TX_EN | RB_UEP7_RX_EN | RB_UEP7_TX_EN; /*!< EP5 EP6 EP7   OUT+IN */
#endif
    CH58x_USBFS_DEV->UEP0_DMA = (uint16_t)(uint32_t)ep0_data_buff[busid];
    CH58x_USBFS_DEV->UEP1_DMA = (uint16_t)(uint32_t)ep1_data_buff[busid];
    CH58x_USBFS_DEV->UEP2_DMA = (uint16_t)(uint32_t)ep2_data_buff[busid];
    CH58x_USBFS_DEV->UEP3_DMA = (uint16_t)(uint32_t)ep3_data_buff[busid];
#if (CONFIG_USBDEV_EP_NUM == 8)
    CH58x_USBFS_DEV->UEP5_DMA = (uint16_t)(uint32_t)ep5_data_buff[busid];
    CH58x_USBFS_DEV->UEP6_DMA = (uint16_t)(uint32_t)ep6_data_buff[busid];
    CH58x_USBFS_DEV->UEP7_DMA = (uint16_t)(uint32_t)ep7_data_buff[busid];
#endif
    CH58x_USBFS_DEV->UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
    CH58x_USBFS_DEV->UEP1_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
    CH58x_USBFS_DEV->UEP2_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
    CH58x_USBFS_DEV->UEP3_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
    CH58x_USBFS_DEV->UEP4_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
#if (CONFIG_USBDEV_EP_NUM == 8)
    CH58x_USBFS_DEV->UEP5_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
    CH58x_USBFS_DEV->UEP6_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
    CH58x_USBFS_DEV->UEP7_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
#endif
    CH58x_USBFS_DEV->USB_DEV_AD = 0x00;

    /*!< Disable USB UDP/UDM pulldown resistance before enabling the USB device. */
    CH58x_USBFS_DEV->UDEV_CTRL = RB_UD_PD_DIS;

    /*!< Start the USB device and DMA, and automatically return to NAK before the interrupt flag is cleared during the interrupt */
    CH58x_USBFS_DEV->USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
    if ((uint32_t)g_usbdev_bus[busid].reg_base == (uint32_t)USB0_BASE) {
        /*!< USB0 */
        R16_PIN_ANALOG_IE |= RB_PIN_USB_IE | RB_PIN_USB_DP_PU;
    } else if ((uint32_t)g_usbdev_bus[busid].reg_base == (uint32_t)USB1_BASE) {
        /*!< USB1 */
        R16_PIN_ANALOG_IE |= RB_PIN_USB2_IE | RB_PIN_USB2_DP_PU;
    }

    CH58x_USBFS_DEV->USB_INT_FG = 0xff; /*!< Clear interrupt flag */
    CH58x_USBFS_DEV->USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
    CH58x_USBFS_DEV->UDEV_CTRL |= RB_UD_PORT_EN; /*!< Allow USB port */

    usb_dc_low_level_init(busid);
    return 0;
}

int usb_dc_deinit(uint8_t busid)
{
    CH58x_USBFS_DEV->USB_INT_EN = 0;
    CH58x_USBFS_DEV->USB_INT_FG = 0xff;
    CH58x_USBFS_DEV->UDEV_CTRL = 0;
    CH58x_USBFS_DEV->USB_CTRL = 0;

    if ((uint32_t)g_usbdev_bus[busid].reg_base == (uint32_t)USB0_BASE) {
        R16_PIN_ANALOG_IE &= ~(RB_PIN_USB_IE | RB_PIN_USB_DP_PU);
    } else if ((uint32_t)g_usbdev_bus[busid].reg_base == (uint32_t)USB1_BASE) {
        R16_PIN_ANALOG_IE &= ~(RB_PIN_USB2_IE | RB_PIN_USB2_DP_PU);
    }

    usb_dc_low_level_deinit(busid);
    return 0;
}

/**
 * @brief            Set address
 * @pre              None
 * @param[in]        address ：8-bit valid address
 * @retval           >=0 success otherwise failure
 */
int usbd_set_address(uint8_t busid, const uint8_t address)
{
    if (address == 0) {
        CH58x_USBFS_DEV->USB_DEV_AD = (CH58x_USBFS_DEV->USB_DEV_AD & RB_UDA_GP_BIT) | address;
    }
    usb_dc_cfg[busid].address = address;
    return 0;
}

int usbd_set_remote_wakeup(uint8_t busid)
{
    return -1;
}

uint8_t usbd_get_port_speed(uint8_t busid)
{
    return USB_SPEED_FULL;
}

/**
 * @brief            Open endpoint
 * @pre              None
 * @param[in]        ep_cfg : Endpoint configuration structure pointer
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep)
{
    /*!< ep id */
    uint8_t epid = USB_EP_GET_IDX(ep->bEndpointAddress);
    USB_ASSERT_MSG(epid < CONFIG_USBDEV_EP_NUM, "Ep addr %02x overflow\r\n", ep->bEndpointAddress);

    /*!< ep max packet length */
    uint8_t mps = USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize);
    /*!< update ep max packet length */
    if (USB_EP_DIR_IS_IN(ep->bEndpointAddress)) {
        /*!< in */
        usb_dc_cfg[busid].ep_in[epid].ep_enable = true;
        usb_dc_cfg[busid].ep_in[epid].mps = mps;
        usb_dc_cfg[busid].ep_in[epid].eptype = USB_GET_ENDPOINT_TYPE(ep->bmAttributes);
        EPn_CTRL(epid) = (EPn_CTRL(epid) & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
    } else if (USB_EP_DIR_IS_OUT(ep->bEndpointAddress)) {
        /*!< out */
        usb_dc_cfg[busid].ep_out[epid].ep_enable = true;
        usb_dc_cfg[busid].ep_out[epid].mps = mps;
        usb_dc_cfg[busid].ep_out[epid].eptype = USB_GET_ENDPOINT_TYPE(ep->bmAttributes);
        EPn_CTRL(epid) = (EPn_CTRL(epid) & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_NAK;
    }
    return 0;
}

/**
 * @brief            Close endpoint
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_close(uint8_t busid, const uint8_t ep)
{
    /*!< ep id */
    uint8_t epid = USB_EP_GET_IDX(ep);
    if (USB_EP_DIR_IS_IN(ep)) {
        /*!< in */
        EPn_CTRL(epid) = (EPn_CTRL(epid) & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
        usb_dc_cfg[busid].ep_in[epid].ep_enable = false;
    } else if (USB_EP_DIR_IS_OUT(ep)) {
        /*!< out */
        EPn_CTRL(epid) = (EPn_CTRL(epid) & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_NAK;
        usb_dc_cfg[busid].ep_out[epid].ep_enable = false;
    }
    return 0;
}

/**
 * @brief            Endpoint setting stall
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_set_stall(uint8_t busid, const uint8_t ep)
{
    /*!< ep id */
    uint8_t epid = USB_EP_GET_IDX(ep);
    if (USB_EP_DIR_IS_OUT(ep)) {
        EPn_SET_RX_STALL(epid);
    } else {
        EPn_SET_TX_STALL(epid);
    }
    return 0;
}

/**
 * @brief            Endpoint clear stall
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_clear_stall(uint8_t busid, const uint8_t ep)
{
    uint8_t epid = USB_EP_GET_IDX(ep);
    if (USB_EP_DIR_IS_OUT(ep)) {
        EPn_CLR_RX_STALL(epid);
    } else {
        EPn_CLR_TX_STALL(epid);
    }
    return 0;
}

/**
 * @brief            Check endpoint status
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @param[out]       stalled ： Outgoing endpoint status
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (USB_EP_DIR_IS_OUT(ep)) {
        *stalled = ((EPn_CTRL(ep_idx) & MASK_UEP_R_RES) == UEP_R_RES_STALL);
    } else {
        *stalled = ((EPn_CTRL(ep_idx) & MASK_UEP_T_RES) == UEP_T_RES_STALL);
    }
    return 0;
}

/**
 * @brief Setup in ep transfer setting and start transfer.
 *
 * This function is asynchronous.
 * This function is similar to uart with tx dma.
 *
 * This function is called to write data to the specified endpoint. The
 * supplied usbd_endpoint_callback function will be called when data is transmitted
 * out.
 *
 * @param[in]  ep        Endpoint address corresponding to the one
 *                       listed in the device configuration table
 * @param[in]  data      Pointer to data to write
 * @param[in]  data_len  Length of the data requested to write. This may
 *                       be zero for a zero length status packet.
 * @return 0 on success, negative errno code on fail.
 */
int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (!data && data_len) {
        return -1;
    }
    if (!usb_dc_cfg[busid].ep_in[ep_idx].ep_enable) {
        return -2;
    }

    usb_dc_cfg[busid].ep_in[ep_idx].xfer_buf = (uint8_t *)data;
    usb_dc_cfg[busid].ep_in[ep_idx].xfer_len = data_len;
    usb_dc_cfg[busid].ep_in[ep_idx].actual_xfer_len = 0;

    if (data_len == 0) {
        /*!< write 0 len data */
        EPn_SET_TX_LEN(ep_idx, 0);
        /*!< enable tx */
        if (usb_dc_cfg[busid].ep_in[ep_idx].eptype != USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            EPn_SET_TX_VALID(ep_idx);
        } else {
            EPn_SET_TX_ISO_VALID(ep_idx);
        }
        /*!< return */
        return 0;
    } else {
        /*!< Not zlp */
        uint32_t write_count = MIN(data_len, usb_dc_cfg[busid].ep_in[ep_idx].mps);
        /*!< write buff */
        memcpy(usb_dc_cfg[busid].ep_in[ep_idx].ep_ram_addr, data, write_count);
        /*!< write real_wt_nums len data */
        EPn_SET_TX_LEN(ep_idx, write_count);
        /*!< enable tx */
        if (usb_dc_cfg[busid].ep_in[ep_idx].eptype != USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            EPn_SET_TX_VALID(ep_idx);
        } else {
            EPn_SET_TX_ISO_VALID(ep_idx);
        }
    }
    return 0;
}

/**
 * @brief Setup out ep transfer setting and start transfer.
 *
 * This function is asynchronous.
 * This function is similar to uart with rx dma.
 *
 * This function is called to read data to the specified endpoint. The
 * supplied usbd_endpoint_callback function will be called when data is received
 * in.
 *
 * @param[in]  ep        Endpoint address corresponding to the one
 *                       listed in the device configuration table
 * @param[in]  data      Pointer to data to read
 * @param[in]  data_len  Max length of the data requested to read.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    if (!data && data_len) {
        return -1;
    }
    if (!usb_dc_cfg[busid].ep_out[ep_idx].ep_enable) {
        return -2;
    }

    usb_dc_cfg[busid].ep_out[ep_idx].xfer_buf = (uint8_t *)data;
    usb_dc_cfg[busid].ep_out[ep_idx].xfer_len = data_len;
    usb_dc_cfg[busid].ep_out[ep_idx].actual_xfer_len = 0;

    if (usb_dc_cfg[busid].ep_out[ep_idx].eptype != USB_ENDPOINT_TYPE_ISOCHRONOUS) {
        EPn_SET_RX_VALID(ep_idx);
    } else {
        EPn_SET_RX_ISO_VALID(ep_idx);
    }
    return 0;
}

/**
 * @brief            USB interrupt processing function
 * @pre              None
 * @param[in]        None
 * @retval           None
 */
void USBD_IRQHandler(uint8_t busid)
{
    volatile uint8_t intflag = 0;
    intflag = CH58x_USBFS_DEV->USB_INT_FG;

    if (intflag & RB_UIF_TRANSFER) {
        if ((CH58x_USBFS_DEV->USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN) {
            uint8_t epid = ((CH58x_USBFS_DEV->USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) & 0x0f);
            switch ((CH58x_USBFS_DEV->USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) & 0xf0) {
                case UIS_TOKEN_IN:
                    if (epid == 0) {
                        uint32_t write_count = MIN(usb_dc_cfg[busid].ep_in[0].xfer_len, usb_dc_cfg[busid].ep_in[0].mps);

                        if (usbd_get_ep0_next_state(busid) == USBD_EP0_STATE_IN_DATA) {
                            CH58x_USBFS_DEV->UEP0_CTRL ^= RB_UEP_T_TOG;
                        }

                        EPn_SET_TX_NAK(0);
                        usb_dc_cfg[busid].ep_in[0].actual_xfer_len = write_count;
                        usb_dc_cfg[busid].ep_in[0].xfer_len = 0;
                        usbd_event_ep_in_complete_handler(busid, 0x80, write_count);

                        if ((usb_dc_cfg[busid].setup.bRequest == USB_REQUEST_SET_ADDRESS) &&
                            (usbd_get_ep0_next_state(busid) == USBD_EP0_STATE_SETUP)) {
                            /*!< Work around CH58x USB IP address update timing issue. */
                            /*!< Fill in the equipment address after status stage */
                            CH58x_USBFS_DEV->USB_DEV_AD = (CH58x_USBFS_DEV->USB_DEV_AD & RB_UDA_GP_BIT) | usb_dc_cfg[busid].address;
                        }
                    } else {
                        EPn_CTRL(epid) ^= RB_UEP_T_TOG;
                        EPn_SET_TX_NAK(epid);
                        if (usb_dc_cfg[busid].ep_in[epid].xfer_len > usb_dc_cfg[busid].ep_in[epid].mps) {
                            /*!< Need start in again */
                            usb_dc_cfg[busid].ep_in[epid].xfer_buf += usb_dc_cfg[busid].ep_in[epid].mps;
                            usb_dc_cfg[busid].ep_in[epid].xfer_len -= usb_dc_cfg[busid].ep_in[epid].mps;
                            usb_dc_cfg[busid].ep_in[epid].actual_xfer_len += usb_dc_cfg[busid].ep_in[epid].mps;
                            uint32_t write_count = MIN(usb_dc_cfg[busid].ep_in[epid].xfer_len, usb_dc_cfg[busid].ep_in[epid].mps);
                            memcpy(usb_dc_cfg[busid].ep_in[epid].ep_ram_addr, usb_dc_cfg[busid].ep_in[epid].xfer_buf, write_count);
                            EPn_SET_TX_LEN(epid, write_count);
                            if (usb_dc_cfg[busid].ep_in[epid].eptype != USB_ENDPOINT_TYPE_ISOCHRONOUS) {
                                EPn_SET_TX_VALID(epid);
                            } else {
                                EPn_SET_TX_ISO_VALID(epid);
                            }
                        } else {
                            usb_dc_cfg[busid].ep_in[epid].actual_xfer_len += usb_dc_cfg[busid].ep_in[epid].xfer_len;
                            usb_dc_cfg[busid].ep_in[epid].xfer_len = 0;
                            usbd_event_ep_in_complete_handler(busid, epid | 0x80, usb_dc_cfg[busid].ep_in[epid].actual_xfer_len);
                        }
                    }
                    break;
                case UIS_TOKEN_OUT:
                    if (!(CH58x_USBFS_DEV->USB_INT_ST & RB_UIS_TOG_OK)) {
                        break;
                    }
                    if (epid == 0) {
                        /*!< ep0 out */
                        CH58x_USBFS_DEV->UEP0_CTRL ^= RB_UEP_R_TOG;
                        EPn_SET_RX_NAK(epid);
                        uint32_t read_count = EPn_GET_RX_LEN(0);
                        if (read_count) {
                            memcpy(usb_dc_cfg[busid].ep_out[epid].xfer_buf, usb_dc_cfg[busid].ep_out[epid].ep_ram_addr, read_count);
                        }

                        usb_dc_cfg[busid].ep_out[0].actual_xfer_len += read_count;
                        usb_dc_cfg[busid].ep_out[0].xfer_len -= read_count;
                        usbd_event_ep_out_complete_handler(busid, 0x00, read_count);
                    } else {
                        EPn_CTRL(epid) ^= RB_UEP_R_TOG;
                        EPn_SET_RX_NAK(epid);
                        uint32_t read_count = EPn_GET_RX_LEN(epid);
                        memcpy(usb_dc_cfg[busid].ep_out[epid].xfer_buf, usb_dc_cfg[busid].ep_out[epid].ep_ram_addr, read_count);
                        usb_dc_cfg[busid].ep_out[epid].xfer_buf += read_count;
                        usb_dc_cfg[busid].ep_out[epid].actual_xfer_len += read_count;
                        usb_dc_cfg[busid].ep_out[epid].xfer_len -= read_count;

                        if ((read_count < usb_dc_cfg[busid].ep_out[epid].mps) || (usb_dc_cfg[busid].ep_out[epid].xfer_len == 0)) {
                            usbd_event_ep_out_complete_handler(busid, epid, usb_dc_cfg[busid].ep_out[epid].actual_xfer_len);
                        } else {
                            if (usb_dc_cfg[busid].ep_out[epid].eptype != USB_ENDPOINT_TYPE_ISOCHRONOUS) {
                                EPn_SET_RX_VALID(epid);
                            } else {
                                EPn_SET_RX_ISO_VALID(epid);
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
            CH58x_USBFS_DEV->USB_INT_FG = RB_UIF_TRANSFER;
        }

        if (CH58x_USBFS_DEV->USB_INT_ST & RB_UIS_SETUP_ACT) {
            /*!< Setup */
            /**
             * Setup the device must respond with ACK, and the next data phase is DATA1
             * If it is sent, the data1 packet will be sent.
             * If it is received, the data1 packet is expected to be received.
             * If it is in, the host will send the data1 out packet to complete the status phase after the in completes.
             * If it is out, the host will send the data1 in packet to complete the status phase after the out completes.
             * SETUP is reported by RB_UIS_SETUP_ACT on CH58x even when EP0 OUT is NAK.
             * Keep EP0 OUT NAK here until the core explicitly arms a data or status OUT stage.
             */
            CH58x_USBFS_DEV->UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_T_RES_NAK | UEP_R_RES_NAK;
            /*!< get setup packet */
            usb_dc_cfg[busid].setup = GET_SETUP_PACKET(usb_dc_cfg[busid].ep_out[0].ep_ram_addr);
            usbd_event_ep0_setup_complete_handler(busid, (uint8_t *)&(usb_dc_cfg[busid].setup));
            CH58x_USBFS_DEV->USB_INT_FG = RB_UIF_TRANSFER;
        }
    } else if (intflag & RB_UIF_BUS_RST) {
        /*!< Reset */
        CH58x_USBFS_DEV->USB_DEV_AD = 0;
        CH58x_USBFS_DEV->UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
        CH58x_USBFS_DEV->UEP1_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
        CH58x_USBFS_DEV->UEP2_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
        CH58x_USBFS_DEV->UEP3_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
        CH58x_USBFS_DEV->UEP4_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
#if (CONFIG_USBDEV_EP_NUM == 8)
        CH58x_USBFS_DEV->UEP5_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
        CH58x_USBFS_DEV->UEP6_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
        CH58x_USBFS_DEV->UEP7_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
#endif
        usbd_event_reset_handler(busid);
        CH58x_USBFS_DEV->USB_INT_FG = RB_UIF_BUS_RST;
    } else if (intflag & RB_UIF_SUSPEND) {
        if (CH58x_USBFS_DEV->USB_MIS_ST & RB_UMS_SUSPEND) {
            /*!< Suspend */
            usbd_event_suspend_handler(busid);
        } else {
            /*!< Wake up */
            usbd_event_resume_handler(busid);
        }
        CH58x_USBFS_DEV->USB_INT_FG = RB_UIF_SUSPEND;
    } else {
        CH58x_USBFS_DEV->USB_INT_FG = intflag;
    }
}

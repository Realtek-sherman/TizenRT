/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *******************************************************************************/
#define _GSPI_IO_C_

#include <drv_types.h>
//#include "hal_data.h"

#if defined(CONFIG_USB_HCI)
	#ifndef MAX_RECVBUF_SZ
		#ifdef PLATFORM_OS_CE
			#define MAX_RECVBUF_SZ (8192+1024) // 8K+1k
		#else
			#ifdef CONFIG_MINIMAL_MEMORY_USAGE
				#define MAX_RECVBUF_SZ (4000) // about 4K
			#else
				#ifdef CONFIG_PLATFORM_MSTAR
					#define MAX_RECVBUF_SZ (8192) // 8K
				#elif defined(CONFIG_PLATFOMR_CUSTOMER_RTOS)||defined(CONFIG_PLATFORM_TIZENRT)
					#define MAX_RECVBUF_SZ MAX_SKB_BUF_SIZE  /* use skb_data_pool as recvbuf */
				#elif defined(CONFIG_PLATFORM_HISILICON)
					#define MAX_RECVBUF_SZ (16384) /* 16k */
				#else
					#define MAX_RECVBUF_SZ (15360) /* 15k<16k */
				#endif
				//#define MAX_RECVBUF_SZ (20480) //20K
				//#define MAX_RECVBUF_SZ (10240) //10K 
				//#define MAX_RECVBUF_SZ (16384) //  16k - 92E RX BUF :16K
				//#define MAX_RECVBUF_SZ (8192+1024) // 8K+1k		
			#endif
		#endif
	#endif //!MAX_RECVBUF_SZ
#endif
#if 0
#ifdef CONFIG_USB_HCI
#include "usb_io_realtek.h" 

#define REALTEK_USB_VENQT_READ		0xC0
#define REALTEK_USB_VENQT_WRITE		0x40
#define RTW_USB_CONTROL_MSG_TIMEOUT	500//ms

int	usb_init_recv_priv(_adapter *padapter, u16 ini_in_buf_sz)
{
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	int	i, res = _SUCCESS;
	struct recv_buf *precvbuf;

#ifdef PLATFORM_LINUX
	tasklet_init(&precvpriv->recv_tasklet,
		     (void(*)(unsigned long))usb_recv_tasklet,
		     (unsigned long)padapter);
#endif /* PLATFORM_LINUX */

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
#ifdef PLATFORM_LINUX
	precvpriv->int_in_urb = rtw_usb_bus_ops.alloc_urb(0, GFP_KERNEL);
	if (precvpriv->int_in_urb == NULL) {
		res = _FAIL;
		DBG_871X("alloc_urb for interrupt in endpoint fail !!!!\n");
		goto exit;
	}
#endif /* PLATFORM_LINUX */
	precvpriv->int_in_buf = rtw_zmalloc(ini_in_buf_sz);
	if (precvpriv->int_in_buf == NULL) {
		res = _FAIL;
		DBG_871X("alloc_mem for interrupt in endpoint fail !!!!\n");
		goto exit;
	}
#endif /* CONFIG_USB_INTERRUPT_IN_PIPE */

	/* init recv_buf */
	rtw_init_queue(&precvpriv->free_recv_buf_queue);
	rtw_init_queue(&precvpriv->recv_buf_pending_queue);
#ifndef CONFIG_USE_USB_BUFFER_ALLOC_RX
	/* this is used only when RX_IOBUF is sk_buff */
	//skb_queue_head_init(&precvpriv->free_recv_skb_queue); //CJ_MASK init_skb_data_pool has already allocate skb
#endif

	DBG_871X("NR_RECVBUFF: %d\n", NR_RECVBUFF);
	DBG_871X("MAX_RECVBUF_SZ: %d\n", MAX_RECVBUF_SZ);
	precvpriv->pallocated_recv_buf = rtw_zmalloc(NR_RECVBUFF * sizeof(struct recv_buf) + 4);
	if (precvpriv->pallocated_recv_buf == NULL) {
		res = _FAIL;
		goto exit;
	}

	precvpriv->precv_buf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(precvpriv->pallocated_recv_buf), 4);

	precvbuf = (struct recv_buf *)precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF ; i++) {
		rtw_init_listhead(&precvbuf->list);

//		rtw_spinlock_init(&precvbuf->recvbuf_lock);

		precvbuf->alloc_sz = MAX_RECVBUF_SZ;

		res = rtw_os_recvbuf_resource_alloc(padapter, precvbuf);
		if (res == _FAIL)
			break;

//		precvbuf->ref_cnt = 0;
		precvbuf->adapter = padapter;

		/* rtw_list_insert_tail(&precvbuf->list, &(precvpriv->free_recv_buf_queue.queue)); */

		precvbuf++;
	}

	precvpriv->free_recv_buf_queue_cnt = NR_RECVBUFF;

#if defined(PLATFORM_LINUX) || defined(PLATFORM_FREEBSD) || defined(PLATFORM_CUSTOMER_RTOS) || defined(PLATFORM_TIZENRT)
	skb_queue_head_init(&precvpriv->rx_skb_queue);

#ifdef CONFIG_RX_INDICATE_QUEUE
	rtw_memset(&precvpriv->rx_indicate_queue, 0, sizeof(struct ifqueue));
	rtw_mutex_init(&precvpriv->rx_indicate_queue.ifq_mtx, "rx_indicate_queue", NULL, MTX_DEF);
#endif /* CONFIG_RX_INDICATE_QUEUE */

#ifdef CONFIG_PREALLOC_RECV_SKB
	{
		int i;
		SIZE_PTR tmpaddr = 0;
		SIZE_PTR alignment = 0;
		struct sk_buff *pskb = NULL;

		DBG_871X("NR_PREALLOC_RECV_SKB: %d\n", NR_PREALLOC_RECV_SKB);
#ifdef CONFIG_FIX_NR_BULKIN_BUFFER
		DBG_871X("Enable CONFIG_FIX_NR_BULKIN_BUFFER\n");
#endif

		for (i = 0; i < NR_PREALLOC_RECV_SKB; i++) {
#ifdef CONFIG_PREALLOC_RX_SKB_BUFFER
			pskb = rtw_alloc_skb_premem(MAX_RECVBUF_SZ);
#else
			pskb = rtw_skb_alloc(MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ);
#endif /* CONFIG_PREALLOC_RX_SKB_BUFFER */

			if (pskb) {
#ifdef PLATFORM_FREEBSD
				pskb->dev = padapter->pifp;
#else
				pskb->dev = padapter->pnetdev;
#endif /* PLATFORM_FREEBSD */

#ifndef CONFIG_PREALLOC_RX_SKB_BUFFER
				tmpaddr = (SIZE_PTR)pskb->data;
				alignment = tmpaddr & (RECVBUFF_ALIGN_SZ - 1);
				skb_reserve(pskb, (RECVBUFF_ALIGN_SZ - alignment));
#endif
				skb_queue_tail(&precvpriv->free_recv_skb_queue, pskb);
			}
		}
	}
#endif /* CONFIG_PREALLOC_RECV_SKB */

#endif /* defined(PLATFORM_LINUX) || defined(PLATFORM_FREEBSD) || defined(PLATFORM_CUSTOMER_RTOS)*/

exit:

	return res;
}

void usb_free_recv_priv(_adapter *padapter, u16 ini_in_buf_sz)
{
	int i;
	struct recv_buf *precvbuf;
	struct recv_priv	*precvpriv = &padapter->recvpriv;

	precvbuf = (struct recv_buf *)precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF ; i++) {
		rtw_os_recvbuf_resource_free(padapter, precvbuf);
		precvbuf++;
	}

	if (precvpriv->pallocated_recv_buf)
		rtw_mfree(precvpriv->pallocated_recv_buf, NR_RECVBUFF * sizeof(struct recv_buf) + 4);

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
#ifdef PLATFORM_LINUX
	if (precvpriv->int_in_urb)
		rtw_usb_bus_ops.free_urb(precvpriv->int_in_urb);
#endif
	if (precvpriv->int_in_buf)
		rtw_mfree(precvpriv->int_in_buf, ini_in_buf_sz);
#endif /* CONFIG_USB_INTERRUPT_IN_PIPE */

#ifdef PLATFORM_LINUX

	if (skb_queue_len(&precvpriv->rx_skb_queue))
		DBG_871X("rx_skb_queue not empty\n");

	rtw_skb_queue_purge(&precvpriv->rx_skb_queue);

	if (skb_queue_len(&precvpriv->free_recv_skb_queue))
		DBG_871X("free_recv_skb_queue not empty, %d\n", skb_queue_len(&precvpriv->free_recv_skb_queue));

#if !defined(CONFIG_USE_USB_BUFFER_ALLOC_RX)
#if defined(CONFIG_PREALLOC_RECV_SKB) && defined(CONFIG_PREALLOC_RX_SKB_BUFFER)
	{
		struct sk_buff *skb;

		while ((skb = skb_dequeue(&precvpriv->free_recv_skb_queue)) != NULL) {
			if (rtw_free_skb_premem(skb) != 0)
				rtw_skb_free(skb);
		}
	}
#else
	rtw_skb_queue_purge(&precvpriv->free_recv_skb_queue);
#endif /* defined(CONFIG_PREALLOC_RX_SKB_BUFFER) && defined(CONFIG_PREALLOC_RECV_SKB) */
#endif /* !defined(CONFIG_USE_USB_BUFFER_ALLOC_RX) */

#endif /* PLATFORM_LINUX */

#ifdef PLATFORM_FREEBSD
	struct sk_buff  *pskb;
	while (NULL != (pskb = skb_dequeue(&precvpriv->rx_skb_queue)))
		rtw_skb_free(pskb);

#if !defined(CONFIG_USE_USB_BUFFER_ALLOC_RX)
	rtw_skb_queue_purge(&precvpriv->free_recv_skb_queue);
#endif

#ifdef CONFIG_RX_INDICATE_QUEUE
	struct mbuf *m;
	for (;;) {
		IF_DEQUEUE(&precvpriv->rx_indicate_queue, m);
		if (m == NULL)
			break;
		rtw_os_pkt_free(m);
	}
	mtx_destroy(&precvpriv->rx_indicate_queue.ifq_mtx);
#endif /* CONFIG_RX_INDICATE_QUEUE */

#endif /* PLATFORM_FREEBSD */
}

unsigned int ffaddr2pipehdl(struct dvobj_priv *pdvobj, u32 addr)
{
	unsigned int pipe = 0, ep_num = 0;
	struct usb_device *pusbd = pdvobj->pusbdev;

	if (addr == RECV_BULK_IN_ADDR)
		pipe =  rtw_usb_bus_ops.rcvbulkpipe(pusbd, pdvobj->RtInPipe[0]);

	else if (addr == RECV_INT_IN_ADDR)
		pipe =  rtw_usb_bus_ops.rcvintpipe(pusbd, pdvobj->RtInPipe[1]);

	else if (addr < HW_QUEUE_ENTRY) {
#ifdef RTW_HALMAC
		/* halmac already translate queue id to bulk out id */
		ep_num = pdvobj->RtOutPipe[addr];
#else
		ep_num = pdvobj->Queue2Pipe[addr];
#endif
		pipe =  rtw_usb_bus_ops.sndbulkpipe(pusbd, ep_num);
	}

	return pipe;
}

int usbctrl_vendorreq(struct dvobj_priv *pdvobj, u8 request, u16 value, u16 index, void *pdata, u16 len, u8 requesttype)
{
	_adapter	*padapter= pdvobj->if1;
	struct usb_device *udev = pdvobj->pusbdev;

	unsigned int pipe;
	int status = 0;
	u32 tmp_buflen = 0;
	u8 reqtype;
	u8 *pIo_buf;
	int vendorreq_times = 0;

#if (defined(CONFIG_RTL8822B) || defined(CONFIG_RTL8821C))
#define REG_ON_SEC 0x00
#define REG_OFF_SEC 0x01
#define REG_LOCAL_SEC 0x02
	u8 current_reg_sec = REG_LOCAL_SEC;
#endif

#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_DYNAMIC_ALLOCATE
	u8 *tmp_buf;
#else /* use stack memory */
	u8 tmp_buf[MAX_USB_IO_CTL_SIZE];
#endif

	if (padapter->bSurpriseRemoved) {
		status = -EPERM;
		goto exit;
	}

	if (len > MAX_VENDOR_REQ_CMD_SIZE) {
		DBG_871X("[%s] Buffer len error ,vendor request failed\n", __FUNCTION__);
		status = -EINVAL;
		goto exit;
	}

#ifdef CONFIG_USB_VENDOR_REQ_MUTEX
	rtw_enter_critical_mutex(&pdvobj->usb_vendor_req_mutex, NULL);
#endif


	/* Acquire IO memory for vendorreq */
#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_PREALLOC
	pIo_buf = pdvobj->usb_vendor_req_buf;
#else
	#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_DYNAMIC_ALLOCATE
	tmp_buf = rtw_malloc((u32) len + ALIGNMENT_UNIT);
	tmp_buflen = (u32)len + ALIGNMENT_UNIT;
	#else /* use stack memory */
	tmp_buflen = MAX_USB_IO_CTL_SIZE;
	#endif

	/* Added by Albert 2010/02/09 */
	/* For mstar platform, mstar suggests the address for USB IO should be 32 bytes alignment. */
	/* Trying to fix it here. */
	pIo_buf = (tmp_buf == NULL) ? NULL : tmp_buf + ALIGNMENT_UNIT - ((SIZE_PTR)(tmp_buf) & 0x1f);
#endif

	if (pIo_buf == NULL) {
		DBG_871X("[%s] pIo_buf == NULL\n", __FUNCTION__);
		status = -ENOMEM;
		goto release_mutex;
	}

	while (++vendorreq_times <= MAX_USBCTRL_VENDORREQ_TIMES) {
		rtw_memset(pIo_buf, 0, len);

		if (requesttype == 0x01) {
			pipe = rtw_usb_bus_ops.rcvctrlpipe(udev, 0);/* read_in */
			reqtype =  REALTEK_USB_VENQT_READ;		
		} else {
			pipe = rtw_usb_bus_ops.sndctrlpipe(udev, 0);/* write_out */
			reqtype =  REALTEK_USB_VENQT_WRITE;		
			rtw_memcpy(pIo_buf, pdata, len);
		}
		status = rtw_usb_bus_ops.control_msg(udev, pipe, request, reqtype, value, index, pIo_buf, len, RTW_USB_CONTROL_MSG_TIMEOUT);

		if (status == len) {  /* Success this control transfer. */
			rtw_reset_continual_io_error(pdvobj);
			if (requesttype == 0x01) {
				/* For Control read transfer, we have to copy the read data from pIo_buf to pdata. */
				rtw_memcpy(pdata, pIo_buf,  len);
			}
		} else { /* error cases */
			DBG_871X("reg 0x%x, usb %s %u fail, status:%d value=0x%x, vendorreq_times:%d\n"
				, value, (requesttype == 0x01) ? "read" : "write" , len, status, *(u32 *)pdata, vendorreq_times);

			if (status < 0) {
				if (status == (-ESHUTDOWN)	|| status == -ENODEV)
					padapter->bSurpriseRemoved = 1;
				else {
					#ifdef DBG_CONFIG_ERROR_DETECT
					{
						HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
						pHalData->srestpriv.Wifi_Error_Status = USB_VEN_REQ_CMD_FAIL;
					}
					#endif
				}
			} else { /* status != len && status >= 0 */
				if (status > 0) {
					if (requesttype == 0x01) {
						/* For Control read transfer, we have to copy the read data from pIo_buf to pdata. */
						rtw_memcpy(pdata, pIo_buf,  len);
					}
				}
			}

			if (rtw_inc_and_chk_continual_io_error(pdvobj) == _TRUE) {
				padapter->bSurpriseRemoved = 1;
				break;
			}

		}

		/* firmware download is checksumed, don't retry */
		if ((value >= FW_START_ADDRESS) || status == len)
			break;

	}

#if (defined(CONFIG_RTL8822B) || defined(CONFIG_RTL8821C))
	if (value < 0xFE00) {
		if (0x00 <= value && value <= 0xff)
			current_reg_sec = REG_ON_SEC;
		else if (0x1000 <= value && value <= 0x10ff)
			current_reg_sec = REG_ON_SEC;
		else
			current_reg_sec = REG_OFF_SEC;
	} else {
		current_reg_sec = REG_LOCAL_SEC;
	}

	if (current_reg_sec == REG_ON_SEC) {
		unsigned int t_pipe = rtw_usb_bus_ops.sndctrlpipe(udev, 0);/* write_out */
		u8 t_len = 1;
		u8 t_req = 0x05;
		u16 t_reg = 0;
		u16 t_index = 0;

		t_reg = 0x4e0;


		if (status == t_len)
			rtw_reset_continual_io_error(pdvobj);
		else
			DBG_871X("reg 0x%x, usb %s %u fail, status:%d\n", t_reg, "write" , t_len, status);

	}
#endif

	/* release IO memory used by vendorreq */
#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_DYNAMIC_ALLOCATE
	rtw_mfree(tmp_buf, tmp_buflen);
#endif

release_mutex:
#ifdef CONFIG_USB_VENDOR_REQ_MUTEX
	rtw_exit_critical_mutex(&pdvobj->usb_vendor_req_mutex, NULL);
#endif
exit:
	return status;

}

u8 usb_read8(struct dvobj_priv *pdvobj, u32 addr, s32 *err)
{
	u8 request;
	u8 requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u8 data = 0;


	request = 0x05;
	requesttype = 0x01;/* read_in */
	index = 0;/* n/a */

	wvalue = (u16)(addr & 0x0000ffff);
	len = 1;
	usbctrl_vendorreq(pdvobj, request, wvalue, index,
			  &data, len, requesttype);


	return data;
}

u16 usb_read16(struct dvobj_priv *pdvobj, u32 addr, s32 *err)
{
	u8 request;
	u8 requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u16 data = 0;


	request = 0x05;
	requesttype = 0x01;/* read_in */
	index = 0;/* n/a */

	wvalue = (u16)(addr & 0x0000ffff);
	len = 2;
	usbctrl_vendorreq(pdvobj, request, wvalue, index,
			  &data, len, requesttype);


	return data;

}

u32 usb_read32(struct dvobj_priv *pdvobj, u32 addr, s32 *err)
{
	u8 request;
	u8 requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u32 data = 0;


	request = 0x05;
	requesttype = 0x01;/* read_in */
	index = 0;/* n/a */

	wvalue = (u16)(addr & 0x0000ffff);
	len = 4;
	usbctrl_vendorreq(pdvobj, request, wvalue, index,
			  &data, len, requesttype);


	return data;
}

int usb_write8(struct dvobj_priv *pdvobj, u32 addr, u8 buf, s32 *err)
{
	u8 request;
	u8 requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u8 data;
	int ret;


	request = 0x05;
	requesttype = 0x00;/* write_out */
	index = 0;/* n/a */

	wvalue = (u16)(addr & 0x0000ffff);
	len = 1;

	data = buf;
	ret = usbctrl_vendorreq(pdvobj, request, wvalue, index,
				&data, len, requesttype);


	return ret;
}

int usb_write16(struct dvobj_priv *pdvobj, u32 addr, u8 buf, s32 *err)
{
	u8 request;
	u8 requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u16 data;
	int ret;


	request = 0x05;
	requesttype = 0x00;/* write_out */
	index = 0;/* n/a */

	wvalue = (u16)(addr & 0x0000ffff);
	len = 2;

	data = buf;
	ret = usbctrl_vendorreq(pdvobj, request, wvalue, index,
				&data, len, requesttype);


	return ret;

}

int usb_write32(struct dvobj_priv *pdvobj, u32 addr, u8 buf, s32 *err)
{
	u8 request;
	u8 requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u32 data;
	int ret;


	request = 0x05;
	requesttype = 0x00;/* write_out */
	index = 0;/* n/a */

	wvalue = (u16)(addr & 0x0000ffff);
	len = 4;
	data = buf;
	ret = usbctrl_vendorreq(pdvobj, request, wvalue, index,
				&data, len, requesttype);


	return ret;

}

int usb_writeN(struct dvobj_priv *pdvobj, u32 addr, u8* buf,u32 length, s32 *err)
{
	u8 request;
	u8 requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u8 data[VENDOR_CMD_MAX_DATA_LEN] = {0};
	int ret;


	request = 0x05;
	requesttype = 0x00;/* write_out */
	index = 0;/* n/a */

	wvalue = (u16)(addr & 0x0000ffff);
	len = length;
	rtw_memcpy(data, buf, len);
	ret = usbctrl_vendorreq(pdvobj, request, wvalue, index,
				buf, len, requesttype);


	return ret;
}

void usb_read_port_cancel(struct dvobj_priv *pdvobj)
{
	int i;
	struct recv_buf *precvbuf;
	_adapter	*padapter= pdvobj->if1;
	precvbuf = (struct recv_buf *)padapter->recvpriv.precv_buf;

	DBG_871X("%s\n", __func__);

	for (i = 0; i < NR_RECVBUFF ; i++) {

		if (precvbuf->purb){
			rtw_usb_bus_ops.kill_urb(precvbuf->purb);
		}
		precvbuf++;
	}

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
	rtw_usb_bus_ops.kill_urb(padapter->recvpriv.int_in_urb);
#endif
}

static void usb_write_port_complete(struct urb *purb, struct pt_regs *regs)
{
	_irqL irqL;
	int i;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)purb->context;
	/* struct xmit_frame *pxmitframe = (struct xmit_frame *)pxmitbuf->priv_data; */
	/* _adapter			*padapter = pxmitframe->padapter; */
	_adapter	*padapter = pxmitbuf->padapter;
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	/* struct pkt_attrib *pattrib = &pxmitframe->attrib; */

	switch (pxmitbuf->flags) {
	case VO_QUEUE_INX:
		pxmitpriv->voq_cnt--;
		break;
	case VI_QUEUE_INX:
		pxmitpriv->viq_cnt--;
		break;
	case BE_QUEUE_INX:
		pxmitpriv->beq_cnt--;
		break;
	case BK_QUEUE_INX:
		pxmitpriv->bkq_cnt--;
		break;
	default:
		break;
	}

	if (RTW_CANNOT_RUN(padapter)) {
		DBG_871X("%s(): TX Warning! bDriverStopped(%s) OR bSurpriseRemoved(%s) pxmitbuf->buf_tag(%x)\n"
			 , __func__
			 , (padapter->bDriverStopped==_TRUE) ? "True" : "False"
			 , (padapter->bSurpriseRemoved==_TRUE) ? "True" : "False"
			 , pxmitbuf->buf_tag);

		goto check_completion;
	}


	if (purb->status == 0) {
	} else {
		DBG_871X("###=> urb_write_port_complete status(%d)\n", purb->status);
		if ((purb->status == -EPIPE) || (purb->status == -EPROTO)) {
			/* usb_clear_halt(pusbdev, purb->pipe);	 */
			/* msleep(10); */
		//	sreset_set_wifi_error_status(padapter, USB_WRITE_PORT_FAIL);
		} else if (purb->status == -EINPROGRESS) {
			goto check_completion;

		} else if (purb->status == -ENOENT) {
			DBG_871X("%s: -ENOENT\n", __func__);
			goto check_completion;

		} else if (purb->status == -ECONNRESET) {
			DBG_871X("%s: -ECONNRESET\n", __func__);
			goto check_completion;

		} else if (purb->status == -ESHUTDOWN) {
			padapter->bDriverStopped=_TRUE;

			goto check_completion;
		} else {
			padapter->bSurpriseRemoved=_TRUE;
			DBG_871X("bSurpriseRemoved=TRUE\n");

			goto check_completion;
		}
	}

	#ifdef DBG_CONFIG_ERROR_DETECT
	{
		HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
		pHalData->srestpriv.last_tx_complete_time = rtw_get_current_time();
	}
	#endif

check_completion:
//	rtw_enter_critical(&pxmitpriv->lock_sctx, &irqL);
//	rtw_sctx_done_err(&pxmitbuf->sctx,
//		purb->status ? RTW_SCTX_DONE_WRITE_PORT_ERR : RTW_SCTX_DONE_SUCCESS);
//	rtw_exit_critical(&pxmitpriv->lock_sctx, &irqL);

	rtw_free_xmitbuf(pxmitpriv, pxmitbuf);

#ifdef CONFIG_XMIT_TASKLET_THREAD
	tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
#else
	//rtw_if_wifi_wakeup_task(&padapter->halXmitThread);
#endif

}

u32 usb_write_port(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 len)
{
	_irqL irqL;
	unsigned int pipe;
	int status;
	u32 ret = _FAIL, bwritezero = _FALSE;
	struct urb*	purb = NULL;
	_adapter	*padapter= pdvobj->if1;
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)buf;
	struct xmit_frame *pxmitframe = (struct xmit_frame *)pxmitbuf->priv_data;
	struct usb_device *pusbd = pdvobj->pusbdev;
	struct pkt_attrib *pattrib = &pxmitframe->attrib;

	if (RTW_CANNOT_RUN(padapter)) {
		DBG_871X(" DBG_TX %s:%d bDriverStopped%s, bSurpriseRemoved:%s\n", __func__, __LINE__
			 ,(padapter->bDriverStopped==_TRUE) ? "True" : "False"
			, (padapter->bSurpriseRemoved==_TRUE) ? "True" : "False");

		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_TX_DENY);
		goto exit;
	}

#ifdef CONFIG_USE_LOCAL_CRITICAL
	def_cri_val();
	save_and_cli();
#else
	rtw_enter_critical(&pxmitpriv->lock, &irqL);
#endif

	switch (addr) {
	case VO_QUEUE_INX:
		pxmitpriv->voq_cnt++;
		pxmitbuf->flags = VO_QUEUE_INX;
		break;
	case VI_QUEUE_INX:
		pxmitpriv->viq_cnt++;
		pxmitbuf->flags = VI_QUEUE_INX;
		break;
	case BE_QUEUE_INX:
		pxmitpriv->beq_cnt++;
		pxmitbuf->flags = BE_QUEUE_INX;
		break;
	case BK_QUEUE_INX:
		pxmitpriv->bkq_cnt++;
		pxmitbuf->flags = BK_QUEUE_INX;
		break;
	case HIGH_QUEUE_INX:
		pxmitbuf->flags = HIGH_QUEUE_INX;
		break;
	default:
		pxmitbuf->flags = MGT_QUEUE_INX;
		break;
	}

#ifdef CONFIG_USE_LOCAL_CRITICAL
	restore_flags();
#else
	rtw_exit_critical(&pxmitpriv->lock, &irqL);
#endif

	purb	= pxmitbuf->pxmit_urb[0];

	/* translate DMA FIFO addr to pipehandle */
#ifdef RTW_HALMAC
	pipe = ffaddr2pipehdl(pdvobj, pxmitbuf->bulkout_id);
#else
	pipe = ffaddr2pipehdl(pdvobj, addr);
#endif

#ifdef CONFIG_REDUCE_USB_TX_INT
	if ((pxmitpriv->free_xmitbuf_cnt % NR_XMITBUFF == 0)
	    || (pxmitbuf->buf_tag > XMITBUF_DATA))
		purb->transfer_flags  &= (~URB_NO_INTERRUPT);
	else {
		purb->transfer_flags  |=  URB_NO_INTERRUPT;
	}
#endif

	rtw_usb_bus_ops.fill_bulk_urb(purb, pusbd, pipe,
			  pxmitframe->buf_addr, /* = pxmitbuf->pbuf */
			  len,
			  usb_write_port_complete,
			  pxmitbuf);/* context is pxmitbuf */

#ifdef CONFIG_USE_USB_BUFFER_ALLOC_TX
	purb->transfer_dma = pxmitbuf->dma_transfer_addr;
	purb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	purb->transfer_flags |= URB_ZERO_PACKET;
#endif /* CONFIG_USE_USB_BUFFER_ALLOC_TX */

#ifdef USB_PACKET_OFFSET_SZ
#if (USB_PACKET_OFFSET_SZ == 0)
	purb->transfer_flags |= URB_ZERO_PACKET;
#endif
#endif

#if 0
	if (bwritezero)
		purb->transfer_flags |= URB_ZERO_PACKET;
#endif

	status = rtw_usb_bus_ops.submit_urb(purb, GFP_ATOMIC);
	if (!status) {
		#ifdef DBG_CONFIG_ERROR_DETECT
		{
			HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
			pHalData->srestpriv.last_tx_time = rtw_get_current_time();
		}
		#endif
	} else {
		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_WRITE_PORT_ERR);
		DBG_871X("usb_write_port, status=%d\n", status);

		switch (status) {
		case -ENODEV:
			padapter->bDriverStopped=_TRUE;
			break;
		default:
			break;
		}
		goto exit;
	}

	ret = _SUCCESS;

exit:
	if (ret != _SUCCESS)
		rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
	return ret;

}

void usb_write_port_cancel(struct dvobj_priv *pdvobj)
{
	int i, j;
	_adapter	*padapter= pdvobj->if1;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)padapter->xmitpriv.pxmitbuf;

	DBG_871X("%s\n", __func__);

	for (i = 0; i < NR_XMITBUFF; i++) {
		for (j = 0; j <MAX_FRAGMENT_NUM; j++) {
			if (pxmitbuf->pxmit_urb[j])
				rtw_usb_bus_ops.kill_urb(pxmitbuf->pxmit_urb[j]);
		}
		pxmitbuf++;
	}

#if USE_XMIT_EXTBUFF
	pxmitbuf = (struct xmit_buf *)padapter->xmitpriv.pxmit_extbuf;
	for (i = 0; i < NR_XMIT_EXTBUFF ; i++) {
		for (j = 0; j < MAX_FRAGMENT_NUM; j++) {
			if (pxmitbuf->pxmit_urb[j])
				rtw_usb_bus_ops.kill_urb(pxmitbuf->pxmit_urb[j]);
		}
		pxmitbuf++;
	}
#endif
}

void usb_init_recvbuf(_adapter *padapter, struct recv_buf *precvbuf)
{

	precvbuf->transfer_len = 0;

	precvbuf->len = 0;

//	precvbuf->ref_cnt = 0;

	if (precvbuf->pbuf) {
		precvbuf->pdata = precvbuf->phead = precvbuf->ptail = precvbuf->pbuf;
		precvbuf->pend = precvbuf->pdata + MAX_RECVBUF_SZ;
	}

}
#if 0
//copy form cj mstar 8188fu
int recvbuf2recvframe(PADAPTER padapter, void *ptr)
{
	u8 *pbuf;
	u8 pkt_cnt = 0;
	u32 pkt_offset;
	s32 transfer_len;
	u8 *pdata;
	union recv_frame *precvframe = NULL;
	struct rx_pkt_attrib *pattrib = NULL;
	PHAL_DATA_TYPE pHalData;
	struct recv_priv *precvpriv;
	_queue *pfree_recv_queue;
	_pkt *pskb;
	struct phy_stat	*pphy_status = NULL;
	u8	rx_report_sz = 0;


	pHalData = GET_HAL_DATA(padapter);
	precvpriv = &padapter->recvpriv;
	pfree_recv_queue = &precvpriv->free_recv_queue;

#ifdef CONFIG_USE_USB_BUFFER_ALLOC_RX
	pskb = NULL;
	transfer_len = (s32)((struct recv_buf *)ptr)->transfer_len;
	pbuf = ((struct recv_buf *)ptr)->pbuf;
#else /* !CONFIG_USE_USB_BUFFER_ALLOC_RX */
	pskb = (_pkt *)ptr;
	transfer_len = (s32)pskb->len;
	pbuf = pskb->data;
#endif /* !CONFIG_USE_USB_BUFFER_ALLOC_RX */

#ifdef CONFIG_USB_RX_AGGREGATION
	pkt_cnt = GET_RX_STATUS_DESC_USB_AGG_PKTNUM_8188F(pbuf);
#endif

	do {
		precvframe = rtw_alloc_recvframe(pfree_recv_queue);
		if (precvframe == NULL) {
			DBG_871X("%s: rtw_alloc_recvframe() failed! RX Drop!\n", __func__);
			goto _exit_recvbuf2recvframe;
		}

		if (transfer_len > 1500)
			rtw_init_listhead(&precvframe->u.hdr.list);
		precvframe->u.hdr.precvbuf = NULL;	/*can't access the precvbuf for new arch. */
		precvframe->u.hdr.len = 0;

		rtl8188f_query_rx_desc_status(precvframe, pbuf);

		pattrib = &precvframe->u.hdr.attrib;

		if ((padapter->registrypriv.mp_mode == 0)
		    && ((pattrib->crc_err) || (pattrib->icv_err))) {
			DBG_871X("%s: RX Warning! crc_err=%d icv_err=%d, skip!\n",
				 __func__, pattrib->crc_err, pattrib->icv_err);

			rtw_free_recvframe(precvframe, pfree_recv_queue);
			goto _exit_recvbuf2recvframe;
		}

		pkt_offset = RXDESC_SIZE + pattrib->drvinfo_sz + pattrib->shift_sz + pattrib->pkt_len;
		if ((pattrib->pkt_len <= 0) || (pkt_offset > transfer_len)) {
			DBG_871X("%s: RX Error! pkt_len=%d pkt_offset=%d transfer_len=%d\n",
				__func__, pattrib->pkt_len, pkt_offset, transfer_len);

			rtw_free_recvframe(precvframe, pfree_recv_queue);
			goto _exit_recvbuf2recvframe;
		}

#ifdef CONFIG_RX_PACKET_APPEND_FCS
		if (check_fwstate(&padapter->mlmepriv, WIFI_MONITOR_STATE) == _FALSE)
			if ((pattrib->pkt_rpt_type == NORMAL_RX) && rtw_hal_rcr_check(padapter, RCR_APPFCS))
				pattrib->pkt_len -= IEEE80211_FCS_LEN;
#endif

		pdata = pbuf + RXDESC_SIZE + pattrib->drvinfo_sz + pattrib->shift_sz;
//===========================

//===========================
//===========================
		if (rtw_os_alloc_recvframe(padapter, precvframe, pdata, pskb) == _FAIL) {
			DBG_871X("%s: RX Error! rtw_os_alloc_recvframe FAIL!\n", __func__);

			rtw_free_recvframe(precvframe, pfree_recv_queue);
			goto _exit_recvbuf2recvframe;
		}
//===========================

		recvframe_put(precvframe, pattrib->pkt_len);

		if (pattrib->pkt_rpt_type == NORMAL_RX)
		{
			pphy_status = pbuf + RXDESC_OFFSET;
				
#ifdef CONFIG_CONCURRENT_MODE
			if(rtw_buddy_adapter_up(padapter))
			{
				if(pre_recv_entry(precvframe, precvbuf, (struct phy_stat*)pphy_status) != _SUCCESS)
				{
					RT_TRACE_F(_module_rtl871x_recv_c_,_drv_info_,
								"recvbuf2recvframe: rtw_recv_entry(precvframe) != _SUCCESS\n");
				}
			}
			else
#endif
			{
				if (pattrib->physt)
					update_recvframe_phyinfo_88f(precvframe, (struct phy_stat*)pphy_status);
				if (rtw_recv_entry(precvframe) != _SUCCESS)
				{
					RT_TRACE_F(_module_rtl871x_recv_c_, _drv_info_, "recvbuf2recvframe: rtw_recv_entry(precvframe) != _SUCCESS\n");
				}
			}
		}
		else {
#ifdef CONFIG_FW_C2H_PKT
			if (pattrib->pkt_rpt_type == C2H_PACKET)
				rtw_hal_c2h_pkt_pre_hdl(padapter, precvframe->u.hdr.rx_data, pattrib->pkt_len);
			else {
				DBG_871X("%s: [WARNNING] RX type(%d) not be handled!\n",
					 __func__, pattrib->pkt_rpt_type);
			}
#endif /* CONFIG_FW_C2H_PKT */
			rtw_free_recvframe(precvframe, pfree_recv_queue);
		}

#ifdef CONFIG_USB_RX_AGGREGATION
		/* jaguar 8-byte alignment */
		pkt_offset = (u16)_RND8(pkt_offset);
		pkt_cnt--;
		pbuf += pkt_offset;
#endif
		transfer_len -= pkt_offset;
		precvframe = NULL;
	} while (transfer_len > 0);

_exit_recvbuf2recvframe:

	return _SUCCESS;
}
#endif

#ifdef CONFIG_USE_USB_BUFFER_ALLOC_RX
void usb_recv_tasklet(void *priv)
{
	struct recv_buf *precvbuf = NULL;
	_adapter	*padapter = (_adapter *)priv;
	struct recv_priv	*precvpriv = &padapter->recvpriv;

	while (NULL != (precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue))) {
		if (RTW_CANNOT_RUN(padapter)) {
			DBG_871X("recv_tasklet => bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
				, (padapter->bDriverStopped==_TRUE)? "True" : "False"
				, (padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
			break;
		}

		recvbuf2recvframe(padapter, precvbuf);

		rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf);
	}
}

void usb_read_port_complete(struct urb *purb, struct pt_regs *regs)
{
	struct recv_buf	*precvbuf = (struct recv_buf *)purb->context;
	_adapter			*padapter = (_adapter *)precvbuf->adapter;
	struct recv_priv	*precvpriv = &padapter->recvpriv;

	ATOMIC_DEC(&(precvpriv->rx_pending_cnt));

	if (RTW_CANNOT_RUN(padapter)) {
		DBG_871X("%s() RX Warning! bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
			 , __func__
			 , (padapter->bDriverStopped==_TRUE)? "True" : "False"
			,(padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
		return;
	}

	if (purb->status == 0) {

		if ((purb->actual_length > MAX_RECVBUF_SZ) || (purb->actual_length < RXDESC_SIZE)) {
			DBG_871X("%s()-%d: urb->actual_length:%u, MAX_RECVBUF_SZ:%u, RXDESC_SIZE:%u\n"
				, __FUNCTION__, __LINE__, purb->actual_length, MAX_RECVBUF_SZ, RXDESC_SIZE);
			rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf);
		} else {
			rtw_reset_continual_io_error(adapter_to_dvobj(padapter));

			precvbuf->transfer_len = purb->actual_length;

			rtw_enqueue_recvbuf(precvbuf, &precvpriv->recv_buf_pending_queue);

			tasklet_schedule(&precvpriv->recv_tasklet);
		}
	} else {

		DBG_871X("###=> usb_read_port_complete => urb.status(%d)\n", purb->status);

		if (rtw_inc_and_chk_continual_io_error(adapter_to_dvobj(padapter)) == _TRUE)
			padapter->bSurpriseRemoved=_TRUE;

		switch (purb->status) {
		case -EINVAL:
		case -EPIPE:
		case -ENODEV:
		case -ESHUTDOWN:
		case -ENOENT:
			padapter->bDriverStopped=_TRUE;
			break;
		case -EPROTO:
		case -EILSEQ:
		case -ETIME:
		case -ECOMM:
		case -EOVERFLOW:
			#ifdef DBG_CONFIG_ERROR_DETECT
			{
				HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
				pHalData->srestpriv.Wifi_Error_Status = USB_READ_PORT_FAIL;
			}
			#endif
			rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf);
			break;
		case -EINPROGRESS:
			DBG_871X("ERROR: URB IS IN PROGRESS!/n");
			break;
		default:
			break;
		}
	}

}

u32 usb_read_port(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 len, struct fifo_more_data *more_data)
{
	int err;
	unsigned int pipe;
	u32 ret = _SUCCESS;
	PURB purb = NULL;
	struct recv_buf	*precvbuf = (struct recv_buf *)buf;
	_adapter		*adapter = pdvobj->if1;
	struct recv_priv	*precvpriv = &adapter->recvpriv;
	struct usb_device	*pusbd = pdvobj->pusbdev;


	if (RTW_CANNOT_RUN(adapter) || (precvbuf == NULL)) {
		return _FAIL;
	}

	usb_init_recvbuf(adapter, precvbuf);

	if (precvbuf->pbuf) {
		ATOMIC_INC(&(precvpriv->rx_pending_cnt));
		purb = precvbuf->purb;

		/* translate DMA FIFO addr to pipehandle */
		pipe = ffaddr2pipehdl(pdvobj, addr);

		rtw_usb_bus_ops.fill_bulk_urb(purb, pusbd, pipe,
			precvbuf->pbuf,
			MAX_RECVBUF_SZ,
			usb_read_port_complete,
			precvbuf);/* context is precvbuf */

		purb->transfer_dma = precvbuf->dma_transfer_addr;
		purb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		err = rtw_usb_bus_ops.submit_urb(purb, GFP_ATOMIC);
		if ((err) && (err != (-EPERM))) {
			DBG_871X("cannot submit rx in-token(err = 0x%08x),urb_status = %d\n", err, purb->status);
			ret = _FAIL;
		}

	}


	return ret;
}
#else	/* CONFIG_USE_USB_BUFFER_ALLOC_RX */
#ifdef CONFIG_RECV_THREAD_MODE
void usb_recv_thread(void *priv)
{
	_pkt			*pskb;
	_adapter		*padapter = (_adapter *)priv;
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct recv_buf	*precvbuf = NULL;
	s32 ret;
	struct task_struct * precvthread = &padapter->recvThread;
	systime Rx_start_time = 0;
	rtw_thread_enter("usb_recv_thread");
	do{
		ret = rtw_down_sema(&padapter->recvThread.wakeup_sema);
		Rx_start_time = rtw_get_current_time();
		if (_FAIL == ret) {
			DBG_8192C("%s: down recv_sema fail!\n", __FUNCTION__);
			//return _FAIL;
			break;
		}

		if(padapter->RxStop)
			break;
		
		if (RTW_CANNOT_RUN(padapter)) {
				DBG_8192C("recv_thread=> bDriverStopped or bSurpriseRemoved\n");
				break;
		}		
		while (NULL != (pskb = skb_dequeue(&precvpriv->rx_skb_queue))) {

			if (RTW_CANNOT_RUN(padapter)) {
				DBG_871X("usb_recv_thread => bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
					, (padapter->bDriverStopped==_TRUE)? "True" : "False"
					, (padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
				#ifdef CONFIG_PREALLOC_RX_SKB_BUFFER
				if (rtw_free_skb_premem(pskb) != 0)
				#endif /* CONFIG_PREALLOC_RX_SKB_BUFFER */
				kfree_skb(pskb);
				break;
			}

			recvbuf2recvframe(padapter, pskb);

			skb_reset_tail_pointer(pskb);
			pskb->len = 0;

		//	skb_queue_tail(&precvpriv->free_recv_skb_queue, pskb);
			kfree_skb(pskb);
			
			precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue);
			if (NULL != precvbuf) {
				precvbuf->pskb = NULL;
				rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (u8 *)precvbuf,0);
			}
		}
	}while(1);
	rtw_up_sema(&precvthread->terminate_sema);
	padapter->RxStop = 2;
	DBG_871X("recv thread exit\n");
}
#else
void usb_recv_tasklet(void *priv)
{
	_pkt			*pskb;
	_adapter		*padapter = (_adapter *)priv;
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct recv_buf	*precvbuf = NULL;

	while (NULL != (pskb = skb_dequeue(&precvpriv->rx_skb_queue))) {

		if (RTW_CANNOT_RUN(padapter)) {
			DBG_871X("recv_tasklet => bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
				, (padapter->bDriverStopped==_TRUE)? "True" : "False"
				, (padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
			#ifdef CONFIG_PREALLOC_RX_SKB_BUFFER
			if (rtw_free_skb_premem(pskb) != 0)
			#endif /* CONFIG_PREALLOC_RX_SKB_BUFFER */
				rtw_skb_free(pskb);
			break;
		}

		recvbuf2recvframe(padapter, pskb);

		skb_reset_tail_pointer(pskb);
		pskb->len = 0;

		skb_queue_tail(&precvpriv->free_recv_skb_queue, pskb);

		precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue);
		if (NULL != precvbuf) {
			precvbuf->pskb = NULL;
			rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf,0);
		}
	}
}
#endif
void usb_read_port_complete(struct urb *purb, struct pt_regs *regs)
{
	struct recv_buf	*precvbuf = (struct recv_buf *)purb->context;
	_adapter			*padapter = (_adapter *)precvbuf->adapter;
	struct recv_priv	*precvpriv = &padapter->recvpriv;

	ATOMIC_DEC(&(precvpriv->rx_pending_cnt));

	if (RTW_CANNOT_RUN(padapter)) {
		DBG_871X("%s() RX Warning! bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
			, __func__
			, (padapter->bDriverStopped==_TRUE)? "True" : "False"
			,(padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
		goto exit;
	}

	if (purb->status == 0) {

		if ((purb->actual_length > MAX_RECVBUF_SZ) || (purb->actual_length < RXDESC_SIZE)) {
			DBG_871X("%s()-%d: urb->actual_length:%u, MAX_RECVBUF_SZ:%u, RXDESC_SIZE:%u\n"
				, __FUNCTION__, __LINE__, purb->actual_length, MAX_RECVBUF_SZ, RXDESC_SIZE);
			rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf,0);
		} else {
			rtw_reset_continual_io_error(adapter_to_dvobj(padapter));

			precvbuf->transfer_len = purb->actual_length;
			skb_put(precvbuf->pskb, purb->actual_length);
			skb_queue_tail(&precvpriv->rx_skb_queue, precvbuf->pskb);

			#ifndef CONFIG_FIX_NR_BULKIN_BUFFER
			if (skb_queue_len(&precvpriv->rx_skb_queue) <= 1)
			#endif
			#ifdef PLATFORM_LINUX
				tasklet_schedule(&precvpriv->recv_tasklet);
			#else
				rtw_wakeup_task(&padapter->recvThread);
			#endif

			precvbuf->pskb = NULL;
			rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf,0);
//			DBG_871X("usb_read_port_complete precvbuf = %p ## skbbuf:%d ## skbdata:%d ## \n", precvbuf, skbbuf_used_num, skbdata_used_num);
		}
	} else {

		DBG_871X("###=> usb_read_port_complete => urb.status(%d)\n", purb->status);

		if (rtw_inc_and_chk_continual_io_error(adapter_to_dvobj(padapter)) == _TRUE)
			padapter->bSurpriseRemoved=_TRUE;

		switch (purb->status) {
		case -EINVAL:
		case -EPIPE:
		case -ENODEV:
		case -ESHUTDOWN:
		case -ENOENT:
			padapter->bDriverStopped=_TRUE;
			break;
		case -EPROTO:
		case -EILSEQ:
		case -ETIME:
		case -ECOMM:
		case -EOVERFLOW:
			#ifdef DBG_CONFIG_ERROR_DETECT
			{
				HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
				pHalData->srestpriv.Wifi_Error_Status = USB_READ_PORT_FAIL;
			}
			#endif
			rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf,0);
			break;
		case -EINPROGRESS:
			DBG_871X("ERROR: URB IS IN PROGRESS!/n");
			break;
		default:
			break;
		}
	}

exit:
	return;
}

u32 usb_read_port(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 len, struct fifo_more_data *more_data)
{
	int err;
	unsigned int pipe;
	u32 ret = _FAIL;
	struct urb* purb = NULL;
	struct recv_buf	*precvbuf = (struct recv_buf *)buf;
	_adapter		*adapter = pdvobj->if1;
	struct recv_priv	*precvpriv = &adapter->recvpriv;
	struct usb_device	*pusbd = pdvobj->pusbdev;
	int recv_skb_size;

	if (RTW_CANNOT_RUN(adapter) || (precvbuf == NULL)) {
		goto exit;
	}

	usb_init_recvbuf(adapter, precvbuf);

	if (precvbuf->pskb == NULL) {
		SIZE_PTR tmpaddr = 0;
		SIZE_PTR alignment = 0;

#if !defined(CONFIG_DONT_CARE_TP)&& !defined(TX_RX_SKBDATA_APRT)
		recv_skb_size = MAX_RECVBUF_SZ;
		precvbuf->pskb = dev_alloc_skb(recv_skb_size, (HAL_INTERFACE_CMD_LEN + RX_RESERV_HEADROOM));
#else
		recv_skb_size = MAX_RECVBUF_SZ;
		precvbuf->pskb = dev_alloc_rx_skb(recv_skb_size, (HAL_INTERFACE_CMD_LEN + RX_RESERV_HEADROOM));
#endif

		if (NULL != precvbuf->pskb){
			tmpaddr = (SIZE_PTR)precvbuf->pskb->data;
			alignment = tmpaddr & (RECVBUFF_ALIGN_SZ - 1);
			skb_reserve(precvbuf->pskb, (RECVBUFF_ALIGN_SZ - alignment));
			goto recv_buf_hook;
		}

		#ifndef CONFIG_FIX_NR_BULKIN_BUFFER
		//precvbuf->pskb = rtw_skb_alloc(MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ);
		#endif

		if (precvbuf->pskb == NULL) {
			if (1)
				DBG_871X("usb_read_port() enqueue precvbuf=%p\n", precvbuf);
			/* enqueue precvbuf and wait for free skb */
			rtw_enqueue_recvbuf(precvbuf, &precvpriv->recv_buf_pending_queue);
			goto exit;
		}

	}

recv_buf_hook:
	precvbuf->phead = precvbuf->pskb->head;
	precvbuf->pdata = precvbuf->pskb->data;
	precvbuf->ptail = skb_tail_pointer(precvbuf->pskb);
	precvbuf->pend = skb_end_pointer(precvbuf->pskb);
	precvbuf->pbuf = precvbuf->pskb->data;

	purb = precvbuf->purb;

	/* translate DMA FIFO addr to pipehandle */
	pipe = ffaddr2pipehdl(pdvobj, addr);

	rtw_usb_bus_ops.fill_bulk_urb(purb, pusbd, pipe,
		precvbuf->pbuf,
		MAX_RECVBUF_SZ,
		usb_read_port_complete,
		precvbuf);

	err = rtw_usb_bus_ops.submit_urb(purb, GFP_ATOMIC);
	if (err && err != (-EPERM)) {
		DBG_871X("cannot submit rx in-token(err = 0x%08x),urb_status = %d\n"
			, err, purb->status);
		goto exit;
	}

	ATOMIC_INC(&(precvpriv->rx_pending_cnt));
	ret = _SUCCESS;

exit:


	return ret;
}
#endif /* CONFIG_USE_USB_BUFFER_ALLOC_RX */

void usb_set_intf_ops(struct _io_ops	*pops)
{
	pops->init_io_priv = NULL;
	pops->write8_endian = NULL;

	pops->_read8 = &usb_read8;
	pops->_read16 = &usb_read16;
	pops->_read32 = &usb_read32;

	pops->_write8 = &usb_write8;
	pops->_write16 = &usb_write16;
	pops->_write32 = &usb_write32;
	pops->_writeN = &usb_writeN;

	pops->read_rx_fifo = &usb_read_port;
	pops->write_tx_fifo = &usb_write_port;
	pops->_read_port_cancel = &usb_read_port_cancel;
	pops->_write_port_cancel = &usb_write_port_cancel;
}
#endif

#else
#include "usb_io_realtek.h"

extern USB_BUS_OPS rtw_usb_bus_ops;


u8 usb_read8(struct dvobj_priv *pdvobj, u32 addr, s32 *err)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	unsigned int len;
	u8 data = 0;
	len = 1;

	rtw_usb_bus_ops.usb_ctrl_req(pusb_data->usb_intf,1,addr,&data,len);

	return data;
}

u16 usb_read16(struct dvobj_priv *pdvobj, u32 addr, s32 *err)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	unsigned int len;
	u8 data_array[2] = {0};
	u16 data = 0;
	len = 2;

	rtw_usb_bus_ops.usb_ctrl_req(pusb_data->usb_intf,1,addr,data_array,len);

	data = data_array[0];
	data |= data_array[1]<<8;

	return data;
}

u32 usb_read32(struct dvobj_priv *pdvobj, u32 addr, s32 *err)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	unsigned int len;
	u8 data_array[4] = {0};
	u32 data = 0;
	len = 4;
	int i;

	rtw_usb_bus_ops.usb_ctrl_req(pusb_data->usb_intf,1,addr,data_array,len);

	data = data_array[0];
	data |= data_array[1]<<8;
	data |= data_array[2]<<16;
	data |= data_array[3]<<24;

	return data;
}




int usb_write8(struct dvobj_priv *pdvobj, u32 addr, u8 buf, s32 *err)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	unsigned int len;
	len = 1;

	return rtw_usb_bus_ops.usb_ctrl_req(pusb_data->usb_intf,0,addr,&buf,len);

}


int usb_write16(struct dvobj_priv *pdvobj, u32 addr, u16 buf, s32 *err)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	unsigned int len;
	len = 2;

	return rtw_usb_bus_ops.usb_ctrl_req(pusb_data->usb_intf,0,addr,&buf,len);

}

int usb_write32(struct dvobj_priv *pdvobj, u32 addr, u32 buf, s32 *err)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	unsigned int len;
	len = 4;
	
	return rtw_usb_bus_ops.usb_ctrl_req(pusb_data->usb_intf,0,addr,&buf,len);

}

int usb_writeN(struct dvobj_priv *pdvobj, u32 addr, u8 *buf,u32 len, s32 *err)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	return rtw_usb_bus_ops.usb_ctrl_req(pusb_data->usb_intf,0,addr,buf,len);
}


void usb_init_recvbuf(_adapter *padapter, struct recv_buf *precvbuf)
{
	precvbuf->transfer_len = 0;

	precvbuf->len = 0;

	if (precvbuf->pbuf) {
		precvbuf->pdata = precvbuf->phead = precvbuf->ptail = precvbuf->pbuf;
		precvbuf->pend = precvbuf->pdata + MAX_RECVBUF_SZ;
	}

}

unsigned int ffaddr2pipehdl(struct dvobj_priv *pdvobj, u32 addr)
{
	unsigned int pipe = 0, ep_num = 0;
	PUSB_DATA pusb_data = NULL;

	pusb_data = &pdvobj->intf_data;

	if (addr == RECV_BULK_IN_ADDR)
		pipe =  rtw_usb_bus_ops.usb_get_bulk_in_pipe(pusb_data->usb_intf, pdvobj->RtInPipe[0]);

	else if (addr < HW_QUEUE_ENTRY) {
		ep_num = pdvobj->Queue2Pipe[addr];
		pipe =  rtw_usb_bus_ops.usb_get_bulk_out_pipe(pusb_data->usb_intf, ep_num);
	}

	return pipe;
}

#ifdef CONFIG_RECV_THREAD_MODE
void usb_recv_thread(void *priv)
{
	_pkt			*pskb;
#ifndef CONFIG_PLATFORM_TIZENRT
	_adapter		*padapter = (_adapter *)priv;
#else
	extern PADAPTER padapter_for_Tizenrt;
	_adapter		*padapter = (_adapter *)padapter_for_Tizenrt;
#endif
	
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct recv_buf	*precvbuf = NULL;
	s32 ret;
	struct task_struct * precvthread = &padapter->recvThread;
	systime Rx_start_time = 0;
	rtw_thread_enter("usb_recv_thread");
	do{
		ret = rtw_down_sema(&padapter->recvThread.wakeup_sema);
		Rx_start_time = rtw_get_current_time();
		if (_FAIL == ret) {
			DBG_8192C("%s: down recv_sema fail!\n", __FUNCTION__);
			//return _FAIL;
			break;
		}

		if(padapter->RxStop)
			break;
		
		if (RTW_CANNOT_RUN(padapter)) {
				DBG_8192C("recv_thread=> bDriverStopped or bSurpriseRemoved\n");
				break;
		}		
		while (NULL != (pskb = skb_dequeue(&precvpriv->rx_skb_queue))) {

			if (RTW_CANNOT_RUN(padapter)) {
				DBG_871X("usb_recv_thread => bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
					, (padapter->bDriverStopped==_TRUE)? "True" : "False"
					, (padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
				#ifdef CONFIG_PREALLOC_RX_SKB_BUFFER
				if (rtw_free_skb_premem(pskb) != 0)
				#endif /* CONFIG_PREALLOC_RX_SKB_BUFFER */
				kfree_skb(pskb);
				break;
			}

			recvbuf2recvframe(padapter, pskb);

			skb_reset_tail_pointer(pskb);
			pskb->len = 0;

		//	skb_queue_tail(&precvpriv->free_recv_skb_queue, pskb);
			kfree_skb(pskb);
			
			precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue);
			if (NULL != precvbuf) {
				precvbuf->pskb = NULL;
				rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (u8 *)precvbuf,0);
			}
		}
	}while(1);
	rtw_up_sema(&precvthread->terminate_sema);
	padapter->RxStop = 2;
	DBG_871X("recv thread exit\n");

_func_exit_;

	rtw_thread_exit();
}
#else
void usb_recv_tasklet(void *priv)
{
	_pkt			*pskb;
	_adapter		*padapter = (_adapter *)priv;
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct recv_buf	*precvbuf = NULL;

	while (NULL != (pskb = skb_dequeue(&precvpriv->rx_skb_queue))) {

		if (RTW_CANNOT_RUN(padapter)) {
			DBG_871X("recv_tasklet => bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
				, (padapter->bDriverStopped==_TRUE)? "True" : "False"
				, (padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
			#ifdef CONFIG_PREALLOC_RX_SKB_BUFFER
			if (rtw_free_skb_premem(pskb) != 0)
			#endif /* CONFIG_PREALLOC_RX_SKB_BUFFER */
				rtw_skb_free(pskb);
			break;
		}

		recvbuf2recvframe(padapter, pskb);

		skb_reset_tail_pointer(pskb);
		pskb->len = 0;

		skb_queue_tail(&precvpriv->free_recv_skb_queue, pskb);

		precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue);
		if (NULL != precvbuf) {
			precvbuf->pskb = NULL;
			rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf,0);
		}
	}
}
#endif

void usb_read_port_complete(void *arg, ssize_t  result)
{
	struct recv_buf	*precvbuf = (struct recv_buf *)arg;
	_adapter			*padapter = (_adapter *)precvbuf->adapter;
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	
	/*************************************************************/
	int q;
	printf("\r\n -------------usb_read_port_complete result= %d \n",result);
	printf("dump packet \n");
	for(q=0;q<result;q++)
	{
		printf("%02x ",*(precvbuf->pbuf+q));
		if((q+1) %16 == 0)
			printf("\n");
		
	}
	printf("\n");
	/*************************************************************/
	//ndbg("\r\n==============>>22222222222222222\r\n");
	if (precvbuf == NULL) {
		return;
	}

	ATOMIC_DEC(&(precvpriv->rx_pending_cnt));

	if (RTW_CANNOT_RUN(padapter)) {
		DBG_871X("%s() RX Warning! bDriverStopped(%s) OR bSurpriseRemoved(%s)\n"
			 , __func__
			 , (padapter->bDriverStopped==_TRUE)? "True" : "False"
			,(padapter->bSurpriseRemoved==_TRUE)? "True" : "False");
		return;
	}

	if ((result > MAX_RECVBUF_SZ) || (result < RXDESC_SIZE)) {
		DBG_871X("usb_read_port_complete: length(%d) is invalid \n",result);
		rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf,0);
	} else {
		rtw_reset_continual_io_error(adapter_to_dvobj(padapter));

		precvbuf->transfer_len = result;
		skb_put(precvbuf->pskb,result);
		skb_queue_tail(&precvpriv->rx_skb_queue, precvbuf->pskb);

		#ifndef CONFIG_FIX_NR_BULKIN_BUFFER
		if (skb_queue_len(&precvpriv->rx_skb_queue) <= 1)
		#endif
		#ifdef PLATFORM_LINUX
			tasklet_schedule(&precvpriv->recv_tasklet);
		#else
			rtw_wakeup_task(&padapter->recvThread);
		#endif

		precvbuf->pskb = NULL;
		rtw_read_port(padapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf,0);
	}

}


u32 usb_read_port(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 len, struct fifo_more_data *more_data)
{
	int err;
	unsigned int pipe;
	u32 ret = _FAIL;
	struct recv_buf	*precvbuf = (struct recv_buf *)buf;
	_adapter		*adapter = pdvobj->if1;
	struct recv_priv	*precvpriv = &adapter->recvpriv;
	int recv_skb_size;
	PUSB_DATA pusb_data;

	//printf("\r\n==========================>MAX_RECVBUF_SZ = %d\r\n",MAX_RECVBUF_SZ);
	//ndbg("\r\n==============>>1111111111111111\r\n");

	if (RTW_CANNOT_RUN(adapter) || (precvbuf == NULL)) {
		goto exit;
	}

	pusb_data = &pdvobj->intf_data;

	usb_init_recvbuf(adapter, precvbuf);

	if (precvbuf->pskb == NULL) {
		SIZE_PTR tmpaddr = 0;
		SIZE_PTR alignment = 0;

#if !defined(CONFIG_DONT_CARE_TP)&& !defined(TX_RX_SKBDATA_APRT)
		recv_skb_size = MAX_RECVBUF_SZ;
		precvbuf->pskb = dev_alloc_skb(recv_skb_size, (HAL_INTERFACE_CMD_LEN + RX_RESERV_HEADROOM));
#else
		recv_skb_size = MAX_RECVBUF_SZ;
		precvbuf->pskb = dev_alloc_rx_skb(recv_skb_size, (HAL_INTERFACE_CMD_LEN + RX_RESERV_HEADROOM));
#endif

		if (NULL != precvbuf->pskb){
			tmpaddr = (SIZE_PTR)precvbuf->pskb->data;
			alignment = tmpaddr & (RECVBUFF_ALIGN_SZ - 1);
			skb_reserve(precvbuf->pskb, (RECVBUFF_ALIGN_SZ - alignment));
			goto recv_buf_hook;
		}

		#ifndef CONFIG_FIX_NR_BULKIN_BUFFER
		//precvbuf->pskb = rtw_skb_alloc(MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ);
		#endif

		if (precvbuf->pskb == NULL) {
			if (1)
				DBG_871X("usb_read_port() enqueue precvbuf=%p\n", precvbuf);
			/* enqueue precvbuf and wait for free skb */
			rtw_enqueue_recvbuf(precvbuf, &precvpriv->recv_buf_pending_queue);
			goto exit;
		}

	}

recv_buf_hook:
	precvbuf->phead = precvbuf->pskb->head;
	precvbuf->pdata = precvbuf->pskb->data;
	precvbuf->ptail = skb_tail_pointer(precvbuf->pskb);
	precvbuf->pend = skb_end_pointer(precvbuf->pskb);
	precvbuf->pbuf = precvbuf->pskb->data;

	/* translate DMA FIFO addr to pipehandle */
	pipe = ffaddr2pipehdl(pdvobj, addr);

	ndbg("\r\npipe = %d\r\n",pipe);

	err = rtw_usb_bus_ops.usb_bulk_in(
		pusb_data->usb_intf,
		pipe,
		precvbuf->pbuf,
		MAX_RECVBUF_SZ,
		usb_read_port_complete,
		precvbuf
	);

	if (err <0) {
		DBG_871X("usb_read_port() bulk ini fail err = %d\n",err);
		goto exit;
	}

	ATOMIC_INC(&(precvpriv->rx_pending_cnt));
	ret = _SUCCESS;

exit:


	return ret;
}

static void usb_write_port_complete(void *arg, ssize_t  result)
{
	_irqL irqL;
	int i;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)arg;
	/* struct xmit_frame *pxmitframe = (struct xmit_frame *)pxmitbuf->priv_data; */
	/* _adapter			*padapter = pxmitframe->padapter; */
	_adapter	*padapter = pxmitbuf->padapter;
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	/* struct pkt_attrib *pattrib = &pxmitframe->attrib; */

	switch (pxmitbuf->flags) {
	case VO_QUEUE_INX:
		pxmitpriv->voq_cnt--;
		break;
	case VI_QUEUE_INX:
		pxmitpriv->viq_cnt--;
		break;
	case BE_QUEUE_INX:
		pxmitpriv->beq_cnt--;
		break;
	case BK_QUEUE_INX:
		pxmitpriv->bkq_cnt--;
		break;
	default:
		break;
	}

	if (RTW_CANNOT_RUN(padapter)) {
		DBG_871X("%s(): TX Warning! bDriverStopped(%s) OR bSurpriseRemoved(%s) pxmitbuf->buf_tag(%x)\n"
			 , __func__
			 , (padapter->bDriverStopped==_TRUE) ? "True" : "False"
			 , (padapter->bSurpriseRemoved==_TRUE) ? "True" : "False"
			 , pxmitbuf->buf_tag);

		goto check_completion;
	}

	#ifdef DBG_CONFIG_ERROR_DETECT
	{
		HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
		pHalData->srestpriv.last_tx_complete_time = rtw_get_current_time();
	}
	#endif

check_completion:
//	rtw_enter_critical(&pxmitpriv->lock_sctx, &irqL);
//	rtw_sctx_done_err(&pxmitbuf->sctx,
//		purb->status ? RTW_SCTX_DONE_WRITE_PORT_ERR : RTW_SCTX_DONE_SUCCESS);
//	rtw_exit_critical(&pxmitpriv->lock_sctx, &irqL);

	rtw_free_xmitbuf(pxmitpriv, pxmitbuf);

#ifdef CONFIG_XMIT_TASKLET_THREAD
	tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
#else
	//rtw_if_wifi_wakeup_task(&padapter->halXmitThread);
#endif

}

u32 usb_write_port(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 len)
{
	PUSB_DATA pusb_data = &pdvobj->intf_data;
	int err = 0;
	_irqL irqL;
	unsigned int pipe;
	u32 ret = _FAIL;
	_adapter	*padapter= pdvobj->if1;
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)buf;
	struct xmit_frame *pxmitframe = (struct xmit_frame *)pxmitbuf->priv_data;
	struct pkt_attrib *pattrib = &pxmitframe->attrib;

	if (RTW_CANNOT_RUN(padapter)) {
		DBG_871X(" DBG_TX %s:%d bDriverStopped%s, bSurpriseRemoved:%s\n", __func__, __LINE__
			 ,(padapter->bDriverStopped==_TRUE) ? "True" : "False"
			, (padapter->bSurpriseRemoved==_TRUE) ? "True" : "False");

		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_TX_DENY);
		goto exit;
	}

#ifdef CONFIG_USE_LOCAL_CRITICAL
	def_cri_val();
	save_and_cli();
#else
	rtw_enter_critical(&pxmitpriv->lock, &irqL);
#endif

	switch (addr) {
	case VO_QUEUE_INX:
		pxmitpriv->voq_cnt++;
		pxmitbuf->flags = VO_QUEUE_INX;
		break;
	case VI_QUEUE_INX:
		pxmitpriv->viq_cnt++;
		pxmitbuf->flags = VI_QUEUE_INX;
		break;
	case BE_QUEUE_INX:
		pxmitpriv->beq_cnt++;
		pxmitbuf->flags = BE_QUEUE_INX;
		break;
	case BK_QUEUE_INX:
		pxmitpriv->bkq_cnt++;
		pxmitbuf->flags = BK_QUEUE_INX;
		break;
	case HIGH_QUEUE_INX:
		pxmitbuf->flags = HIGH_QUEUE_INX;
		break;
	default:
		pxmitbuf->flags = MGT_QUEUE_INX;
		break;
	}

#ifdef CONFIG_USE_LOCAL_CRITICAL
	restore_flags();
#else
	rtw_exit_critical(&pxmitpriv->lock, &irqL);
#endif

	/* translate DMA FIFO addr to pipehandle */
#ifdef RTW_HALMAC
	pipe = ffaddr2pipehdl(pdvobj, pxmitbuf->bulkout_id);
#else
	pipe = ffaddr2pipehdl(pdvobj, addr);
#endif

	err = rtw_usb_bus_ops.usb_bulk_out(
		pusb_data->usb_intf,
		pipe,
		pxmitframe->buf_addr,
	       len,
	       usb_write_port_complete,
	       pxmitbuf
	);

	if (err < 0) {
		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_WRITE_PORT_ERR);
		goto exit;
	} else {
		#ifdef DBG_CONFIG_ERROR_DETECT
		{
			HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
			pHalData->srestpriv.last_tx_time = rtw_get_current_time();
		}
		#endif
	}

	ret = _SUCCESS;

exit:
	if (ret != _SUCCESS)
		rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
	return ret;

}

int	usb_init_recv_priv(_adapter *padapter, u16 ini_in_buf_sz)
{
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	int i, res = _SUCCESS;
	struct recv_buf *precvbuf;

#ifdef PLATFORM_LINUX
	tasklet_init(&precvpriv->recv_tasklet,
			 (void(*)(unsigned long))usb_recv_tasklet,
			 (unsigned long)padapter);
#endif /* PLATFORM_LINUX */

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
#ifdef PLATFORM_LINUX
	precvpriv->int_in_urb = rtw_usb_bus_ops.alloc_urb(0, GFP_KERNEL);
	if (precvpriv->int_in_urb == NULL) {
		res = _FAIL;
		DBG_871X("alloc_urb for interrupt in endpoint fail !!!!\n");
		goto exit;
	}
#endif /* PLATFORM_LINUX */
	precvpriv->int_in_buf = rtw_zmalloc(ini_in_buf_sz);
	if (precvpriv->int_in_buf == NULL) {
		res = _FAIL;
		DBG_871X("alloc_mem for interrupt in endpoint fail !!!!\n");
		goto exit;
	}
#endif /* CONFIG_USB_INTERRUPT_IN_PIPE */

	/* init recv_buf */
	rtw_init_queue(&precvpriv->free_recv_buf_queue);
	rtw_init_queue(&precvpriv->recv_buf_pending_queue);
#ifndef CONFIG_USE_USB_BUFFER_ALLOC_RX
	/* this is used only when RX_IOBUF is sk_buff */
	//skb_queue_head_init(&precvpriv->free_recv_skb_queue); //CJ_MASK init_skb_data_pool has already allocate skb
#endif

	DBG_871X("NR_RECVBUFF: %d\n", NR_RECVBUFF);
	DBG_871X("MAX_RECVBUF_SZ: %d\n", MAX_RECVBUF_SZ);
	precvpriv->pallocated_recv_buf = rtw_zmalloc(NR_RECVBUFF * sizeof(struct recv_buf) + 4);
	if (precvpriv->pallocated_recv_buf == NULL) {
		res = _FAIL;
		goto exit;
	}

	precvpriv->precv_buf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(precvpriv->pallocated_recv_buf), 4);

	precvbuf = (struct recv_buf *)precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF ; i++) {
		rtw_init_listhead(&precvbuf->list);

//		rtw_spinlock_init(&precvbuf->recvbuf_lock);

		precvbuf->alloc_sz = MAX_RECVBUF_SZ;

		res = rtw_os_recvbuf_resource_alloc(padapter, precvbuf);
		if (res == _FAIL)
			break;

//		precvbuf->ref_cnt = 0;
		precvbuf->adapter = padapter;

		/* rtw_list_insert_tail(&precvbuf->list, &(precvpriv->free_recv_buf_queue.queue)); */

		precvbuf++;
	}

	precvpriv->free_recv_buf_queue_cnt = NR_RECVBUFF;

#if defined(PLATFORM_LINUX) || defined(PLATFORM_FREEBSD) || defined(PLATFORM_CUSTOMER_RTOS) || defined(PLATFORM_TIZENRT)
	skb_queue_head_init(&precvpriv->rx_skb_queue);

#ifdef CONFIG_RX_INDICATE_QUEUE
	rtw_memset(&precvpriv->rx_indicate_queue, 0, sizeof(struct ifqueue));
	rtw_mutex_init(&precvpriv->rx_indicate_queue.ifq_mtx, "rx_indicate_queue", NULL, MTX_DEF);
#endif /* CONFIG_RX_INDICATE_QUEUE */

#ifdef CONFIG_PREALLOC_RECV_SKB
	{
		int i;
		SIZE_PTR tmpaddr = 0;
		SIZE_PTR alignment = 0;
		struct sk_buff *pskb = NULL;

		DBG_871X("NR_PREALLOC_RECV_SKB: %d\n", NR_PREALLOC_RECV_SKB);
#ifdef CONFIG_FIX_NR_BULKIN_BUFFER
		DBG_871X("Enable CONFIG_FIX_NR_BULKIN_BUFFER\n");
#endif

		for (i = 0; i < NR_PREALLOC_RECV_SKB; i++) {
#ifdef CONFIG_PREALLOC_RX_SKB_BUFFER
			pskb = rtw_alloc_skb_premem(MAX_RECVBUF_SZ);
#else
			pskb = rtw_skb_alloc(MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ);
#endif /* CONFIG_PREALLOC_RX_SKB_BUFFER */

			if (pskb) {
#ifdef PLATFORM_FREEBSD
				pskb->dev = padapter->pifp;
#else
				pskb->dev = padapter->pnetdev;
#endif /* PLATFORM_FREEBSD */

#ifndef CONFIG_PREALLOC_RX_SKB_BUFFER
				tmpaddr = (SIZE_PTR)pskb->data;
				alignment = tmpaddr & (RECVBUFF_ALIGN_SZ - 1);
				skb_reserve(pskb, (RECVBUFF_ALIGN_SZ - alignment));
#endif
				skb_queue_tail(&precvpriv->free_recv_skb_queue, pskb);
			}
		}
	}
#endif /* CONFIG_PREALLOC_RECV_SKB */

#endif /* defined(PLATFORM_LINUX) || defined(PLATFORM_FREEBSD) || defined(PLATFORM_CUSTOMER_RTOS)*/

exit:

	return res;
}

void usb_free_recv_priv(_adapter *padapter, u16 ini_in_buf_sz)
{
	int i;
	struct recv_buf *precvbuf;
	struct recv_priv	*precvpriv = &padapter->recvpriv;

	precvbuf = (struct recv_buf *)precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF ; i++) {
		rtw_os_recvbuf_resource_free(padapter, precvbuf);
		precvbuf++;
	}

	if (precvpriv->pallocated_recv_buf)
		rtw_mfree(precvpriv->pallocated_recv_buf, NR_RECVBUFF * sizeof(struct recv_buf) + 4);

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
#ifdef PLATFORM_LINUX
	if (precvpriv->int_in_urb)
		rtw_usb_bus_ops.free_urb(precvpriv->int_in_urb);
#endif
	if (precvpriv->int_in_buf)
		rtw_mfree(precvpriv->int_in_buf, ini_in_buf_sz);
#endif /* CONFIG_USB_INTERRUPT_IN_PIPE */

#ifdef PLATFORM_LINUX

	if (skb_queue_len(&precvpriv->rx_skb_queue))
		DBG_871X("rx_skb_queue not empty\n");

	rtw_skb_queue_purge(&precvpriv->rx_skb_queue);

	if (skb_queue_len(&precvpriv->free_recv_skb_queue))
		DBG_871X("free_recv_skb_queue not empty, %d\n", skb_queue_len(&precvpriv->free_recv_skb_queue));

#if !defined(CONFIG_USE_USB_BUFFER_ALLOC_RX)
#if defined(CONFIG_PREALLOC_RECV_SKB) && defined(CONFIG_PREALLOC_RX_SKB_BUFFER)
	{
		struct sk_buff *skb;

		while ((skb = skb_dequeue(&precvpriv->free_recv_skb_queue)) != NULL) {
			if (rtw_free_skb_premem(skb) != 0)
				rtw_skb_free(skb);
		}
	}
#else
	rtw_skb_queue_purge(&precvpriv->free_recv_skb_queue);
#endif /* defined(CONFIG_PREALLOC_RX_SKB_BUFFER) && defined(CONFIG_PREALLOC_RECV_SKB) */
#endif /* !defined(CONFIG_USE_USB_BUFFER_ALLOC_RX) */

#endif /* PLATFORM_LINUX */

#ifdef PLATFORM_FREEBSD
	struct sk_buff  *pskb;
	while (NULL != (pskb = skb_dequeue(&precvpriv->rx_skb_queue)))
		rtw_skb_free(pskb);

#if !defined(CONFIG_USE_USB_BUFFER_ALLOC_RX)
	rtw_skb_queue_purge(&precvpriv->free_recv_skb_queue);
#endif

#ifdef CONFIG_RX_INDICATE_QUEUE
	struct mbuf *m;
	for (;;) {
		IF_DEQUEUE(&precvpriv->rx_indicate_queue, m);
		if (m == NULL)
			break;
		rtw_os_pkt_free(m);
	}
	mtx_destroy(&precvpriv->rx_indicate_queue.ifq_mtx);
#endif /* CONFIG_RX_INDICATE_QUEUE */

#endif /* PLATFORM_FREEBSD */
}

void usb_read_port_cancel(struct dvobj_priv *pdvobj)
{
	PUSB_DATA pusb_data;

	pusb_data = &pdvobj->intf_data;

	rtw_usb_bus_ops.usb_cancel_bulk_in(pusb_data->usb_intf);
}

void usb_write_port_cancel(struct dvobj_priv *pdvobj)
{
	PUSB_DATA pusb_data;

	pusb_data = &pdvobj->intf_data;

	rtw_usb_bus_ops.usb_cancel_bulk_out(pusb_data->usb_intf);
}

void usb_set_intf_ops(struct _io_ops	*pops)
{
	pops->init_io_priv = NULL;
	pops->write8_endian = NULL;

	pops->_read8 = &usb_read8;
	pops->_read16 = &usb_read16;
	pops->_read32 = &usb_read32;

	pops->_write8 = &usb_write8;
	pops->_write16 = &usb_write16;
	pops->_write32 = &usb_write32;
	pops->_writeN = &usb_writeN;

	pops->read_rx_fifo = &usb_read_port;
	pops->write_tx_fifo = &usb_write_port;
	pops->_read_port_cancel = &usb_read_port_cancel;
	pops->_write_port_cancel = &usb_write_port_cancel;
	printf("\r\n-------------------------->\r\n");

	//if(bmutext_init == 0){
	//	bmutext_init = 1;
		//rtw_mutex_init(&usb_host_mutex);
	//}
}

#endif


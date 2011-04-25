/*
 * USB Attached SCSI (UAS) protocol driver
 * Copyright Luben Tuikov, 2010, 2011
 *
 * This driver allows you to connect to UAS devices and use them as
 * SCSI devices.
 *
 * Distributed under the terms of the GNU GPL version 2.
 */

#ifdef UASP_DEBUG
#if !defined(DEBUG) && !defined(CONFIG_DYNAMIC_DEBUG)
#define DEBUG
#endif
#endif /* UASP_DEBUG */

#define DRIVER_NAME "uasp"

#include <linux/blkdev.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/usb/storage.h>
#include <linux/completion.h>

#include <scsi/scsi.h>
#include <scsi/scsi_dbg.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_tcq.h>

int MaxNumStreams = -1;

module_param(MaxNumStreams, int, 0444);
MODULE_PARM_DESC(MaxNumStreams, "\n"
	"\tSome host controllers and/or devices report a larger number of\n"
	"\tstreams that they in fact support. This parameter allows you\n"
	"\tto limit the number of streams this driver will request the XHCI\n"
	"\tHCD to allocate. If set to -1, the default value, then this driver\n"
	"\twill use the value reported by the attached device. Else the\n"
	"\tnumber of streams will be limited to the minimum reported by the\n"
	"\tattached device and this value. Valid values are -1, default,\n"
	"\tand 1 to 0xFFEF.");

/* Information unit types
 */
#define IU_CMD   1
#define IU_SENSE 3
#define IU_RESP  4
#define IU_TMF   5
#define IU_RRDY  6
#define IU_WRDY  7

#define IU_CMD_LEN	32
#define IU_SENSE_LEN	16
#define IU_RESP_LEN	8
#define IU_TMF_LEN	16
#define IU_RRDY_LEN	4
#define IU_WRDY_LEN	4

#define MAX_SENSE_DATA_LEN 252

#define STAT_IU_LEN	(IU_SENSE_LEN + MAX_SENSE_DATA_LEN)

#define GET_IU_ID(__Ptr)	((__Ptr)[0])
#define GET_IU_TAG(__Ptr)	((__Ptr)[2] << 8 | (__Ptr)[3])

/* SENSE IU
 */
#define GET_IU_STATQUAL(__Ptr)	((__Ptr)[4] << 8 | (__Ptr)[5])
#define GET_IU_STATUS(__Ptr)	((__Ptr)[6])
#define GET_IU_LENGTH(__Ptr)	((__Ptr)[14] << 8 | (__Ptr)[15])

/* RESPONSE IU
 */
#define GET_IU_RESPONSE(__Ptr)	((__Ptr)[4] << 24 | (__Ptr)[5] << 16 | \
				 (__Ptr)[6] << 8 | (__Ptr)[7])

/* Task management
 */
#define TMF_ABORT_TASK		1
#define TMF_ABORT_TASK_SET	2
#define TMF_CLEAR_TASK_SET	4
#define TMF_LU_RESET		8
#define TMF_IT_NEXUS_RESET	0x10
#define TMF_CLEAR_ACA		0x40
#define TMF_QUERY_TASK		0x80
#define TMF_QUERY_TASK_SET	0x81
#define TMF_QUERY_ASYNC_EVENT   0x82

#define TMR_RESPONSE_CODE_MASK	0xFF
#define TMR_RESPONSE_CODE_SHIFT 0
#define TMR_RESPONSE_INFO_MASK  0xFFFFFF00
#define TMR_RESPONSE_INFO_SHIFT 8

#define TMR_RESPONSE_CODE(__Val)					\
	(((__Val) & TMR_RESPONSE_CODE_MASK) >> TMR_RESPONSE_CODE_SHIFT)

#define TMR_RESPONSE_INFO(__Val)					\
	(((__Val) & TMR_RESPONSE_INFO_MASK) >> TMR_RESPONSE_INFO_SHIFT)

#define TMR_COMPLETE 0
#define TMR_IIU      2
#define TMR_UNSUPP   4
#define TMR_FAILED   5
#define TMR_SUCC     8
#define TMR_ILUN     9
#define TMR_OLAP     0xA

/* Pipe types
 */
#define CMD_PIPE_ID     1
#define STAT_PIPE_ID    2
#define DATAIN_PIPE_ID  3
#define DATAOUT_PIPE_ID 4

/* 1 control (default) and 4 as per UAS.
 */
#define UASP_NUM_EPS	(1+4)

#define FIRST_EP_WSTREAMS STAT_PIPE_ID
#define LAST_EP_WSTREAMS  DATAOUT_PIPE_ID
#define NUM_EPS_WSTREAMS  (LAST_EP_WSTREAMS - FIRST_EP_WSTREAMS + 1)

/* Task attribute
 */
#define UASP_TASK_SIMPLE  0
#define UASP_TASK_HOQ     1
#define UASP_TASK_ORDERED 2
#define UASP_TASK_ACA     4

struct uasp_tag_data {
	int first_tag_avail;
	int num_tags;
	int last_tag_alloc;		  /* last tag allocated */
	struct scsi_cmnd **tag_cmd;
	unsigned long *tag_map;
};

/* Target device port information
 */
struct uasp_tport_info {
	struct usb_interface *iface;
	struct usb_device *udev;
	unsigned cmd_pipe, status_pipe, datain_pipe, dataout_pipe;
	struct usb_host_endpoint *eps[UASP_NUM_EPS];
	unsigned use_streams:1;
	int max_streams;	  /* streams supported */
	int num_streams;	  /* usable streams [1, num_streams] */
	int max_cmds;		  /* max cmds we can queue */

	struct uasp_tag_data tag_data; /* Per I_T nexus */
};

/* Current task management information
 */
struct uasp_lu_info {
	struct completion tmf_completion;
	unsigned int tmf_resp;
	void *freed_urb;
};

/* Command information
 */
struct uasp_cmd_info {
	int tag;
	struct urb *cmd_urb;
	struct urb *status_urb;
	struct urb *datain_urb;
	struct urb *dataout_urb;
};

#define UASP_CMD_INFO(__Scmd) ((struct uasp_cmd_info *)(&(__Scmd)->SCp))
#define UASP_TPORT_INFO(__Scmd) \
	((struct uasp_tport_info *)((__Scmd)->device->host->hostdata[0]))
#define SDEV_TPORT_INFO(__Sdev)  \
	((struct uasp_tport_info *)((__Sdev)->host->hostdata[0]))
#define SDEV_LU_INFO(__Sdev) ((struct uasp_lu_info *)((__Sdev)->hostdata))

/* ---------- SCSI Tags processing ---------- */

static int uasp_tag_alloc(struct uasp_tag_data *td, struct scsi_cmnd *cmd)
{
	int tag;

	if (td->last_tag_alloc >= td->num_tags - 1)
		td->last_tag_alloc = td->first_tag_avail - 1;

	do {
		tag = find_next_zero_bit(td->tag_map, td->num_tags,
					 td->last_tag_alloc + 1);
		if (tag >= td->num_tags)
			return -1;

	} while (test_and_set_bit_lock(tag, td->tag_map));

	td->last_tag_alloc = tag;
	td->tag_cmd[tag] = cmd;

	return tag;
}

static int uasp_tag_lookup(struct uasp_tag_data *td, struct scsi_cmnd *cmd)
{
	struct uasp_cmd_info *ci = UASP_CMD_INFO(cmd);

	if (td->tag_cmd[ci->tag] == cmd && test_bit(ci->tag, td->tag_map))
		return ci->tag;
	else
		return -1;
}

static inline struct scsi_cmnd *uasp_cmd_lookup(struct uasp_tag_data *td,
						int tag)
{
	return td->tag_cmd[tag];
}

static void uasp_tag_free(struct uasp_tag_data *td, int tag)
{
	td->tag_cmd[tag] = NULL;
	clear_bit(tag, td->tag_map);
}

static void uasp_tag_releaseall(struct uasp_tag_data *td)
{
	int i;

	for (i = 0; i < td->num_tags; i++)
		td->tag_cmd[i] = NULL;
	bitmap_zero(td->tag_map, td->num_tags);
}

static int uasp_alloc_tags(struct uasp_tag_data *td, int first_tag,
			   int last_tag, gfp_t gfp)
{
	/* Don't bother with offsets in the case where first_tag is a
	 * large number in order to preserve memory space. This makes
	 * the code simpler. first_tag for most protocols is either 0
	 * or 1. In UAS it is 1.
	 */
	td->first_tag_avail = first_tag;
	td->num_tags = last_tag+1;
	td->last_tag_alloc = td->first_tag_avail - 1;

	td->tag_cmd = kzalloc(td->num_tags * sizeof(*td->tag_cmd), gfp);
	if (td->tag_cmd == NULL)
		return -ENOMEM;

	td->tag_map = kzalloc(BITS_TO_LONGS(td->num_tags) * sizeof(long), gfp);
	if (td->tag_map == NULL) {
		kfree(td->tag_cmd);
		return -ENOMEM;
	}

	return 0;
}

static void uasp_free_tags(struct uasp_tag_data *td)
{
	kfree(td->tag_cmd);
	kfree(td->tag_map);
	memset(td, 0, sizeof(*td));
}

/* ---------- IU processors ---------- */

static void uasp_sense(struct urb *urb, struct scsi_cmnd *cmd, u16 tag)
{
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);
	unsigned char *iu = urb->transfer_buffer;
	struct scsi_device *sdev = cmd->device;
	int id = GET_IU_ID(iu);

	if (id == IU_RESP) {
		unsigned int resp = GET_IU_RESPONSE(iu);

		if (TMR_RESPONSE_CODE(resp) == TMR_IIU)
			cmd->result = DID_ABORT << 16;
		else {
			cmd->result = DID_ERROR << 16;
			dev_err(&SDEV_TPORT_INFO(sdev)->udev->dev,
				"%s: doesn't know how to handle TMR 0x%08x\n",
				__func__, resp);
		}
	} else if (likely(id == IU_SENSE)) {
		cmd->result = GET_IU_STATUS(iu);

		if (urb->actual_length > IU_SENSE_LEN) {
			unsigned slen = GET_IU_LENGTH(iu);

			if (urb->actual_length >= IU_SENSE_LEN + slen) {
				slen = min(slen,
					   (unsigned) SCSI_SENSE_BUFFERSIZE);
			} else {
				unsigned dlen = slen;

				slen = min(urb->actual_length - IU_SENSE_LEN,
					   (unsigned) SCSI_SENSE_BUFFERSIZE);

				dev_err(&SDEV_TPORT_INFO(sdev)->udev->dev,
					"%s: short SENSE IU by %d bytes, "
					"using %d/%d bytes of sense data\n",
					__func__,
					IU_SENSE_LEN + slen
					- urb->actual_length, slen, dlen);
			}
			memcpy(cmd->sense_buffer, iu + IU_SENSE_LEN, slen);
		}
	} else {
		/* Impossible! */
		cmd->result = DID_ERROR << 16;
		dev_err(&SDEV_TPORT_INFO(sdev)->udev->dev,
			"%s: called with IU ID 0x%02x\n", __func__, id);
	}

	dev_dbg(&SDEV_TPORT_INFO(sdev)->udev->dev,
		"%s: completing %p result 0x%02x\n",
		__func__, cmd, cmd->result);

	cmd->scsi_done(cmd);
	usb_free_urb(urb);
	uasp_tag_free(&tpinfo->tag_data, tag);
}

/* High-Speed devices only
 */
static void uasp_xfer_data(struct urb *urb, struct scsi_cmnd *cmd,
			   struct uasp_tport_info *tpinfo,
			   enum dma_data_direction dir, int tag)
{
	struct uasp_cmd_info *cmdinfo = UASP_CMD_INFO(cmd);
	int res = 0;

	res = usb_submit_urb(urb, GFP_ATOMIC);

	if (res == 0) {
		if (dir == DMA_FROM_DEVICE)
			res = usb_submit_urb(cmdinfo->datain_urb, GFP_ATOMIC);
		else
			res = usb_submit_urb(cmdinfo->dataout_urb,GFP_ATOMIC);
	}

	dev_dbg(&tpinfo->udev->dev, "%s: cmd:%p tag:%d res:%d\n",
		__func__, cmd, cmdinfo->tag, res);
}

#ifdef UASP_DEBUG

static const char *id_to_str[] = {
	[0 ... 0xFF] = "Unknown",
	[1] = "Command",
	[3] = "Sense",
	[4] = "Response",
	[5] = "Task Management",
	[6] = "RRDY",
	[7] = "WRDY",
};

#endif /* UASP_DEBUG */

/**
 * uasp_stat_done -- Status pipe urb completion
 * @urb: the URB that completed
 *
 * Anything we expect to come back on the status pipe
 * comes here.
 */
static void uasp_stat_done(struct urb *urb)
{
	unsigned char *iu = urb->transfer_buffer;
	struct scsi_device *sdev = urb->context;
	struct uasp_tport_info *tpinfo = SDEV_TPORT_INFO(sdev);
	struct scsi_cmnd *cmd = NULL;
	int id = GET_IU_ID(iu);
	int tag =  GET_IU_TAG(iu);

	dev_dbg(&urb->dev->dev, "%s: %s IU (0x%02x) tag:%d urb:%p status:%d\n",
		__func__, id_to_str[id], id, tag, urb, urb->status);

	if (urb->status ||
	    (id != IU_SENSE && id != IU_RESP &&
	     id != IU_RRDY && id != IU_WRDY)) {
		usb_free_urb(urb);
		return;
	}

	if (id == IU_RESP) {
		struct uasp_lu_info *luinfo = SDEV_LU_INFO(sdev);
		unsigned char *riu = urb->transfer_buffer;
		unsigned int resp = GET_IU_RESPONSE(riu);

		if (likely(TMR_RESPONSE_CODE(resp) != TMR_IIU)) {
			luinfo->tmf_resp = resp;
			luinfo->freed_urb = urb;
			usb_free_urb(urb);
			complete(&luinfo->tmf_completion);
			return;
		}
	}

	cmd = uasp_cmd_lookup(&tpinfo->tag_data, tag);
	if (unlikely(cmd == NULL))
		goto Out_no_cmd;
	
	switch (id) {
	case IU_SENSE:
	case IU_RESP:
		uasp_sense(urb, cmd, tag);
		break;
	case IU_RRDY:
		uasp_xfer_data(urb, cmd, tpinfo, DMA_FROM_DEVICE, tag);
		break;
	case IU_WRDY:
		uasp_xfer_data(urb, cmd, tpinfo, DMA_TO_DEVICE, tag);
		break;
	default:
		uasp_tag_free(&tpinfo->tag_data, tag);
		usb_free_urb(urb);
		break;
	}

	return;

 Out_no_cmd:
	dev_dbg(&urb->dev->dev, "%s: No command with tag %d!?\n", __func__,tag);
	uasp_tag_free(&tpinfo->tag_data, tag);
	usb_free_urb(urb);
}

static void uasp_data_done(struct urb *urb)
{
	struct scsi_data_buffer *sdb = urb->context;

	dev_dbg(&urb->dev->dev, "%s: urb:%p\n", __func__, urb);

	if (urb->status == 0)
		sdb->resid = sdb->length - urb->actual_length;
	else
		dev_err(&urb->dev->dev, "%s: URB status %d\n",
			__func__, urb->status);

	usb_free_urb(urb);
}

/* ---------- URB allocators and submission ---------- */

/**
 * uasp_fill_cmdp_urb -- Fill in a command pipe urb
 * @urb: the urb
 * @tpinfo: the UAS device info struct
 * @transfer_buffer: the IU
 * @buffer_length: the size of the IU
 *
 * A unified place to initialize a command pipe urb so that
 * all command pipe urbs are freed in the same manner.
 */
static void uasp_fill_cmdp_urb(struct urb *urb, struct uasp_tport_info *tpinfo,
			       void *transfer_buffer, int buffer_length)
{
	usb_fill_bulk_urb(urb, tpinfo->udev, tpinfo->cmd_pipe,
			  transfer_buffer, buffer_length,
			  usb_free_urb, NULL);
	urb->transfer_flags |= URB_FREE_BUFFER;
}

/**
 * uasp_fill_statp_urb -- Fill in a status pipe urb
 * @urb: the urb
 * @dev: the scsi device
 * @transfer_buffer: the transfer buffer of size STAT_IU_LEN
 * @tag: the tag
 *
 * The callback is responsible to free/recycle the urb and the
 * transfer buffer.
 */
static void uasp_fill_statp_urb(struct urb *urb, struct scsi_device *sdev,
				void *transfer_buffer, int tag)
{
	struct uasp_tport_info *tpinfo = SDEV_TPORT_INFO(sdev);

	usb_fill_bulk_urb(urb, tpinfo->udev, tpinfo->status_pipe,
			  transfer_buffer, STAT_IU_LEN,
			  uasp_stat_done, sdev);
	urb->transfer_flags |= URB_FREE_BUFFER;
	if (tpinfo->use_streams)
		urb->stream_id = tag;
}

static struct urb *uasp_alloc_data_urb(struct uasp_tport_info *tpinfo,
				       gfp_t gfp, unsigned int data_pipe,
				       u16 tag, struct scsi_data_buffer *sdb)
{
	struct usb_device *udev = tpinfo->udev;
	struct urb *urb;

	urb = usb_alloc_urb(0, gfp);
	if (urb == NULL)
		goto Out;

	usb_fill_bulk_urb(urb, udev, data_pipe, NULL, sdb->length,
			  uasp_data_done, sdb);
	if (tpinfo->use_streams)
		urb->stream_id = tag;
	urb->num_sgs = udev->bus->sg_tablesize ? sdb->table.nents : 0;
	urb->sg = sdb->table.sgl;
 Out:
	return urb;
}

static struct urb *uasp_alloc_status_urb(struct scsi_cmnd *cmd, gfp_t gfp)
{
	unsigned char *siu;
	struct urb *urb;

	urb = usb_alloc_urb(0, gfp);
	if (urb == NULL)
		return NULL;

	siu = kzalloc(STAT_IU_LEN, gfp);
	if (siu == NULL)
		goto Out_free;

	uasp_fill_statp_urb(urb, cmd->device, siu, UASP_CMD_INFO(cmd)->tag);

	return urb;
 Out_free:
	usb_free_urb(urb);
	return NULL;
}

static struct urb *uasp_alloc_cmd_urb(struct scsi_cmnd *cmd, gfp_t gfp)
{
	struct uasp_cmd_info *cmdinfo = UASP_CMD_INFO(cmd);
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);
	struct scsi_device *sdev = cmd->device;
	struct urb *urb;
	struct scsi_lun slun;
	unsigned char *ciu;
	int len;

	urb = usb_alloc_urb(0, gfp);
	if (urb == NULL)
		return NULL;

	len = cmd->cmd_len + 16;
	if (len < IU_CMD_LEN)
		len = IU_CMD_LEN;
	else if (len > IU_CMD_LEN)
		len = ALIGN(len, 4);

	ciu = kzalloc(len, gfp);
	if (ciu == NULL)
		goto Free;

	ciu[0] = IU_CMD;
	ciu[2] = cmdinfo->tag >> 8;
	ciu[3] = cmdinfo->tag;
	ciu[4] = UASP_TASK_SIMPLE;
	ciu[6] = len - IU_CMD_LEN;
	int_to_scsilun(sdev->lun, &slun);
	memcpy(&ciu[8], slun.scsi_lun, 8);
	memcpy(&ciu[16], cmd->cmnd, cmd->cmd_len);

	uasp_fill_cmdp_urb(urb, tpinfo, ciu, len);

	return urb;
 Free:
	usb_free_urb(urb);
	return NULL;
}

static int uasp_alloc_urbs(struct scsi_cmnd *cmd, gfp_t gfp)
{
	struct uasp_cmd_info *cmdinfo = UASP_CMD_INFO(cmd);
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);

	cmdinfo->status_urb = NULL;
	cmdinfo->datain_urb = NULL;
	cmdinfo->dataout_urb = NULL;
	cmdinfo->cmd_urb = NULL;

	cmdinfo->status_urb = uasp_alloc_status_urb(cmd, gfp);
	cmdinfo->cmd_urb = uasp_alloc_cmd_urb(cmd, gfp);
	if (cmdinfo->cmd_urb == NULL || cmdinfo->status_urb == NULL)
		goto Out_err1;

	cmdinfo->status_urb->ep = tpinfo->eps[STAT_PIPE_ID];
	cmdinfo->cmd_urb->ep = tpinfo->eps[CMD_PIPE_ID];

	switch (cmd->sc_data_direction) {
	case DMA_BIDIRECTIONAL:
	case DMA_TO_DEVICE:
		cmdinfo->dataout_urb =
			uasp_alloc_data_urb(tpinfo, gfp,
					    tpinfo->dataout_pipe,
					    cmdinfo->tag,
					    scsi_out(cmd));
		if (cmdinfo->dataout_urb == NULL)
			goto Out_err2;

		cmdinfo->dataout_urb->ep = tpinfo->eps[DATAOUT_PIPE_ID];

		if (cmd->sc_data_direction != DMA_BIDIRECTIONAL)
			break;
		else {
	case DMA_FROM_DEVICE:
		cmdinfo->datain_urb =
			uasp_alloc_data_urb(tpinfo, gfp,
					    tpinfo->datain_pipe,
					    cmdinfo->tag,
					    scsi_in(cmd));
		if (cmdinfo->datain_urb == NULL)
			goto Out_err2;
		
		cmdinfo->datain_urb->ep = tpinfo->eps[DATAIN_PIPE_ID];

		}
		break;
	case DMA_NONE:
		break;
	}

	dev_dbg(&tpinfo->udev->dev,
		"urbs: c:%p di:%p do:%p s:%p\n",
		cmdinfo->cmd_urb, cmdinfo->datain_urb,
		cmdinfo->dataout_urb, cmdinfo->status_urb);

	return 0;
	
 Out_err2:
	usb_free_urb(cmdinfo->datain_urb);
	usb_free_urb(cmdinfo->dataout_urb);
 Out_err1:
	usb_free_urb(cmdinfo->cmd_urb);
	usb_free_urb(cmdinfo->status_urb);
	return -ENOMEM;
}

static int uasp_submit_urbs(struct scsi_cmnd *cmd, gfp_t gfp)
{
	struct uasp_cmd_info *cmdinfo = UASP_CMD_INFO(cmd);
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);
	int res;

	dev_dbg(&tpinfo->udev->dev, "%s: cmd:%p (0x%02x) tag:%d\n",
		__func__, cmd, cmd->cmnd[0], cmdinfo->tag);

	res = usb_submit_urb(cmdinfo->status_urb, gfp);
	if (res) {
		dev_err(&tpinfo->udev->dev,
			"error submitting status urb (%d)\n", res);
		return res;
	}

	if (cmdinfo->datain_urb && tpinfo->use_streams) {
		res = usb_submit_urb(cmdinfo->datain_urb, gfp);
		if (res) {
			dev_err(&tpinfo->udev->dev,
				"error submitting datain urb (%d)\n", res);
			return res;
		}
	}

	if (cmdinfo->dataout_urb && tpinfo->use_streams) {
		res = usb_submit_urb(cmdinfo->dataout_urb, gfp);
		if (res) {
			dev_err(&tpinfo->udev->dev,
				"error submitting dataout urb (%d)\n", res);
			return res;
		}
	}

	res = usb_submit_urb(cmdinfo->cmd_urb, gfp);
	if (res) {
		dev_err(&tpinfo->udev->dev,
			"error submitting cmd urb (%d)\n", res);
		return res;
	}

	return 0;
}

static void uasp_free_urbs(struct scsi_cmnd *cmd)
{
	struct uasp_cmd_info *cmdinfo = UASP_CMD_INFO(cmd);
	int res;

	if (cmdinfo->cmd_urb) {
		res = usb_unlink_urb(cmdinfo->cmd_urb);
		if (res != -EINPROGRESS)
			usb_free_urb(cmdinfo->cmd_urb);
	}
	if (cmdinfo->status_urb) {
		res = usb_unlink_urb(cmdinfo->status_urb);
		if (res != -EINPROGRESS)
			usb_free_urb(cmdinfo->status_urb);
	}
	if (cmdinfo->datain_urb) {
		res = usb_unlink_urb(cmdinfo->datain_urb);
		if (res != -EINPROGRESS)
			usb_free_urb(cmdinfo->datain_urb);
	}
	if (cmdinfo->dataout_urb) {
		res = usb_unlink_urb(cmdinfo->dataout_urb);
		if (res != -EINPROGRESS)
			usb_free_urb(cmdinfo->dataout_urb);
	}
}

static int uasp_queuecommand_lck(struct scsi_cmnd *cmd,
				 void (*done)(struct scsi_cmnd *))
{
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);
	struct uasp_cmd_info *cmdinfo = UASP_CMD_INFO(cmd);
	int res;

	BUILD_BUG_ON(sizeof(struct uasp_cmd_info) >sizeof(struct scsi_pointer));

	cmdinfo->tag = uasp_tag_alloc(&tpinfo->tag_data, cmd);
	if (cmdinfo->tag < 0) {
		cmd->result = DID_ABORT << 16;
		done(cmd);
		return 0;
	}

	cmd->scsi_done = done;

	res = uasp_alloc_urbs(cmd, GFP_ATOMIC);
	if (res) {
		cmd->result = DID_ABORT << 16;
		goto Out_err;
	}

	res = uasp_submit_urbs(cmd, GFP_ATOMIC);
	if (res) {
		cmd->result = DID_NO_CONNECT << 16;
		uasp_free_urbs(cmd);
		goto Out_err;
	}
	
	dev_dbg(&UASP_TPORT_INFO(cmd)->udev->dev,
		"%s: cmd:%p (0x%02x) tag:%d lun:%d res:%d\n",
		__func__, cmd, cmd->cmnd[0], cmdinfo->tag,
		cmd->device->lun, res);

	return 0;
Out_err:
	uasp_tag_free(&tpinfo->tag_data, cmdinfo->tag);
	done(cmd);
	return 0;
}

static DEF_SCSI_QCMD(uasp_queuecommand);

/* ---------- Error Recovery ---------- */

static int uasp_alloc_tmf_urb(struct urb **urb, struct uasp_tport_info *tpinfo,
			      u8 tmf, u16 ttbm, unsigned char *lun, gfp_t gfp)
{
	unsigned char *tiu;
	int tag;
	
	*urb = usb_alloc_urb(0, gfp);
	if (*urb == NULL)
		return -ENOMEM;

	tiu = kzalloc(IU_TMF_LEN, gfp);
	if (tiu == NULL)
		goto Out_err;

	tag = uasp_tag_alloc(&tpinfo->tag_data, NULL);
	if (tag < 0) {
		kfree(tiu);
		goto Out_err;
	}

	tiu[0] = IU_TMF;
	tiu[2] = tag >> 8;
	tiu[3] = tag;
	tiu[4] = tmf;
	tiu[6] = ttbm >> 8;
	tiu[7] = ttbm;
	memcpy(&tiu[8], lun, 8);

	uasp_fill_cmdp_urb(*urb, tpinfo, tiu, IU_TMF_LEN);

	return 0;
 Out_err:
	usb_free_urb(*urb);
	return -ENOMEM;
}

static int uasp_alloc_resp_urb(struct urb **urb, struct scsi_device *sdev,
			       int tag, gfp_t gfp)
{
	unsigned char *resp;

	*urb = usb_alloc_urb(0, gfp);
	if (*urb == NULL)
		return -ENOMEM;

	resp = kzalloc(STAT_IU_LEN, gfp);
	if (resp == NULL)
		goto Out_free;

	uasp_fill_statp_urb(*urb, sdev, resp, tag);

	return 0;
 Out_free:
	usb_free_urb(*urb);
	return -ENOMEM;
}

#define UASP_TMF_TIMEOUT	(5*HZ)

/**
 * uasp_do_tmf -- Execute the desired TMF
 * @sdev: the device to which we should send the TMF
 * @tmf:  the task management function to execute
 * @ttbm: the tag of task to be managed, or 0
 *
 * This function returns a negative value on error (-ENOMEM, etc), or
 * an integer with bytes 3, 2 and 1 being the high to low byte of the
 * Additional Response Information field and byte 0 being the Response
 * code of the Response IU.
 * 
 * If the response code is 0xFF, then the TMF timed out.
 */
static int uasp_do_tmf(struct scsi_device *sdev, u8 tmf, u8 ttbm, gfp_t gfp)
{
	struct uasp_tport_info *tpinfo = SDEV_TPORT_INFO(sdev);
	struct uasp_lu_info *luinfo = SDEV_LU_INFO(sdev);
	struct urb *tmf_urb = NULL;
	struct urb *resp_urb = NULL;
	unsigned char *tiu;
	int tag;
	struct scsi_lun slun;
	int    res, res1;

	/* scsi_dev should contain u8[8] for a LUN, not an unsigned int!
	 */
	int_to_scsilun(sdev->lun, &slun);
	res = uasp_alloc_tmf_urb(&tmf_urb, tpinfo, tmf, ttbm, slun.scsi_lun,
				 gfp);
	if (res)
		return -ENOMEM;

	tiu = tmf_urb->transfer_buffer;
	tag = GET_IU_TAG(tiu);
	res = uasp_alloc_resp_urb(&resp_urb, sdev, tag, gfp);
	if (res) {
		usb_free_urb(tmf_urb);
		uasp_tag_free(&tpinfo->tag_data, tag);
		return -ENOMEM;
	}

	init_completion(&luinfo->tmf_completion);
	luinfo->tmf_resp = 0xFFFFFFFF;
	luinfo->freed_urb = NULL;

	dev_dbg(&tpinfo->udev->dev, "tmf_urb:%p resp_urb:%p\n",
		tmf_urb, resp_urb);

	res = usb_submit_urb(resp_urb, gfp);
	if (res) {
		dev_err(&tpinfo->udev->dev, "error submitting resp urb (%d)\n",
			res);
		goto Free_urbs;
	}

	res = usb_submit_urb(tmf_urb, gfp);
	if (res) {
		dev_err(&tpinfo->udev->dev, "error submitting tmf urb (%d)\n",
			res);
		goto Free_urbs;
	}

	wait_for_completion_timeout(&luinfo->tmf_completion, UASP_TMF_TIMEOUT);

	if (luinfo->tmf_resp != 0xFFFFFFFF) {
		res = luinfo->tmf_resp;
	} else {
		res = 0xFF;
	}

	if (luinfo->freed_urb != resp_urb)
		usb_kill_urb(resp_urb);

	luinfo->freed_urb = NULL;

	uasp_tag_free(&tpinfo->tag_data, tag);

	return res;

 Free_urbs:
	res1 = usb_unlink_urb(tmf_urb);
	if (res1 != -EINPROGRESS)
		usb_free_urb(tmf_urb);

	res1 = usb_unlink_urb(resp_urb);
	if (res1 != -EINPROGRESS)
		usb_free_urb(resp_urb);

	uasp_tag_free(&tpinfo->tag_data, tag);

	return res;
}

static int uasp_abort_task(struct scsi_cmnd *cmd)
{
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);
	struct scsi_device *sdev = cmd->device;
	int res;
	int ttbm;

	ttbm = uasp_tag_lookup(&tpinfo->tag_data, cmd);
	if (ttbm < 0) {
		dev_err(&tpinfo->udev->dev,
			"cmd: %p is done or was never sent\n", cmd);
		return FAILED;
	}

	res = uasp_do_tmf(sdev, TMF_ABORT_TASK, ttbm, GFP_ATOMIC);

	dev_dbg(&tpinfo->udev->dev,
		"%s: cmd:%p (0x%02x) tag:%d resp:0x%08x\n",
		__func__, cmd, cmd->cmnd[0], ttbm, res);

	switch (TMR_RESPONSE_CODE(res)) {
	case TMR_COMPLETE:
	case TMR_SUCC:
		uasp_tag_free(&tpinfo->tag_data, ttbm);
		return SUCCESS;
	default:
		return FAILED;
	}
}

static int uasp_lu_reset(struct scsi_cmnd *cmd)
{
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);
	struct uasp_tag_data *td = &tpinfo->tag_data;
	int res, i;

	res = uasp_do_tmf(cmd->device, TMF_LU_RESET, 0, GFP_ATOMIC);

	for (i = 0;
	     (i = find_next_bit(td->tag_map, td->num_tags, i)) < td->num_tags;
	     i++) {
		struct scsi_cmnd *xc = uasp_cmd_lookup(td, i);

		if (xc != NULL && xc->device->lun == cmd->device->lun)
			uasp_tag_free(td, i);
	}

	dev_dbg(&tpinfo->udev->dev, "%s: %s %s: result: %d\n", __func__,
		dev_driver_string(&cmd->device->sdev_gendev),
		dev_name(&cmd->device->sdev_gendev), res);

	return res;
}

static int uasp_IT_nexus_reset(struct scsi_cmnd *cmd)
{
	struct uasp_tport_info *tpinfo = UASP_TPORT_INFO(cmd);
	struct uasp_tag_data *td = &tpinfo->tag_data;
	int res;

	res = uasp_do_tmf(cmd->device, TMF_IT_NEXUS_RESET, 0, GFP_ATOMIC);
	uasp_tag_releaseall(td);

	dev_dbg(&tpinfo->udev->dev, "%s: result: %d\n", __func__, res);

	return res;
}

static int uasp_transport_reset(struct scsi_cmnd *cmd)
{
	struct scsi_device *sdev = cmd->device;
	struct uasp_tport_info *tpinfo = SDEV_TPORT_INFO(sdev);
	struct usb_device *udev = tpinfo->udev;
	int res;

	res = usb_lock_device_for_reset(udev, tpinfo->iface);
	if (res == 0) {
		res = usb_reset_device(udev);
		uasp_tag_releaseall(&tpinfo->tag_data);
		usb_unlock_device(udev);
	} else {
		dev_err(&udev->dev, "%s: failed to lock dev (%d)\n", __func__,
			res);
		return FAILED;
	}

	dev_dbg(&udev->dev, "%s: result: %d\n", __func__, res);

	if (res == 0)
		return SUCCESS;

	return FAILED;
}

/* ---------- SCSI Host support ---------- */

static int uasp_slave_alloc(struct scsi_device *sdev)
{
	sdev->hostdata = kzalloc(sizeof(struct uasp_lu_info), GFP_ATOMIC);
	if (sdev->hostdata == NULL)
		return -ENOMEM;

	return 0;
}

static int uasp_slave_configure(struct scsi_device *sdev)
{
	struct uasp_tport_info *tpinfo = SDEV_TPORT_INFO(sdev);

	scsi_set_tag_type(sdev, 0);
	scsi_activate_tcq(sdev, tpinfo->max_cmds);

	return 0;
}

static void uasp_slave_destroy(struct scsi_device *sdev)
{
	kfree(sdev->hostdata);
}

static struct scsi_host_template uasp_host_template = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.queuecommand = uasp_queuecommand,
	.slave_alloc = uasp_slave_alloc,
	.slave_configure = uasp_slave_configure,
	.slave_destroy = uasp_slave_destroy,
	.eh_abort_handler = uasp_abort_task,
	.eh_device_reset_handler = uasp_lu_reset,
	.eh_target_reset_handler = uasp_IT_nexus_reset,
	.eh_bus_reset_handler = uasp_transport_reset,
	.can_queue = 1,
	.cmd_per_lun = 1,
	.this_id = -1,
	.sg_tablesize = SG_NONE,
	.max_sectors = SCSI_DEFAULT_MAX_SECTORS,
	.use_clustering = ENABLE_CLUSTERING,
	.skip_settle_delay = 1,
	.ordered_tag = 1,
};

/* ---------- USB related ---------- */

static const struct usb_device_id uasp_usb_ids[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, USB_SC_SCSI, USB_PR_BULK)},
	{ USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, USB_SC_SCSI, USB_PR_UAS) },
	{ }
};
MODULE_DEVICE_TABLE(usb, uasp_usb_ids);

static int uasp_ep_conf(struct uasp_tport_info *tpinfo)
{
	struct usb_device *udev = tpinfo->udev;
	struct usb_interface *iface = tpinfo->iface;
	struct usb_host_endpoint *epa = iface->cur_altsetting->endpoint;
	int numep = iface->cur_altsetting->desc.bNumEndpoints;
	int i, res, num_tags;

	for (i = 0; i < numep; i++) {
		unsigned char *desc = epa[i].extra;
		int len = epa[i].extralen;

		for ( ; len > 1; len -= desc[0], desc += desc[0]) {
			unsigned pid = desc[2];

			if (desc[0] != 4 || desc[1] != USB_DT_PIPE_USAGE)
				continue;
			else if (CMD_PIPE_ID <= pid && pid <= DATAOUT_PIPE_ID) {
				tpinfo->eps[pid] = &epa[i];
				break;
			}
		}
	}

	if (tpinfo->eps[CMD_PIPE_ID] == NULL ||
	    tpinfo->eps[STAT_PIPE_ID] == NULL ||
	    tpinfo->eps[DATAIN_PIPE_ID] == NULL ||
	    tpinfo->eps[DATAOUT_PIPE_ID] == NULL) {
		dev_err(&udev->dev, "%s: one or more endpoints are missing\n",
			__func__);
		return -ENODEV;
	}

	tpinfo->cmd_pipe = usb_sndbulkpipe(udev,
			 tpinfo->eps[CMD_PIPE_ID]->desc.bEndpointAddress);
	tpinfo->status_pipe = usb_rcvbulkpipe(udev,
			 tpinfo->eps[STAT_PIPE_ID]->desc.bEndpointAddress);
	tpinfo->datain_pipe = usb_rcvbulkpipe(udev,
			 tpinfo->eps[DATAIN_PIPE_ID]->desc.bEndpointAddress);
	tpinfo->dataout_pipe = usb_sndbulkpipe(udev,
			 tpinfo->eps[DATAOUT_PIPE_ID]->desc.bEndpointAddress);

	if (udev->speed == USB_SPEED_SUPER) {
		int max_streams;

		for (i = FIRST_EP_WSTREAMS; i <= LAST_EP_WSTREAMS; i++) {
			if (tpinfo->max_streams == 0)
				tpinfo->max_streams =
		   USB_SS_MAX_STREAMS(tpinfo->eps[i]->ss_ep_comp.bmAttributes);
			else
				tpinfo->max_streams = min(tpinfo->max_streams,
		   USB_SS_MAX_STREAMS(tpinfo->eps[i]->ss_ep_comp.bmAttributes));
		}
		
		if (tpinfo->max_streams <= 1) {
			dev_err(&udev->dev, "%s: no streams\n", __func__);
			return -ENODEV;
		}
		
		tpinfo->use_streams = 1;
		if (1 <= MaxNumStreams && MaxNumStreams <= 0xFFEF)
			max_streams = min(MaxNumStreams, tpinfo->max_streams);
		else
			max_streams = tpinfo->max_streams;

		tpinfo->num_streams =
			usb_alloc_streams(iface,
					  &tpinfo->eps[FIRST_EP_WSTREAMS],
					  NUM_EPS_WSTREAMS, max_streams,
					  GFP_ATOMIC);
		if (tpinfo->num_streams <= 0) {
			dev_err(&udev->dev,
				"%s: Couldn't allocate %d streams (%d)\n",
				__func__, tpinfo->max_streams,
				tpinfo->num_streams);
			return -ENODEV;
		}
		num_tags = tpinfo->num_streams;
		dev_info(&udev->dev, "streams:%d allocated streams:%d\n",
			 tpinfo->max_streams, tpinfo->num_streams);
	} else {
		tpinfo->use_streams = 0;
		num_tags = 32;
	}

	/* Reserve a tag for TMF per target port. Ideally we'd reserve
	 * a tag for TMF per LU, not per target port, but if the
	 * number of Logical Units (say 33) is greater than the number
	 * of streams supported (say 32) then there would be no tags
	 * left for commands.
	 */
	tpinfo->max_cmds = num_tags - 1;
	res = uasp_alloc_tags(&tpinfo->tag_data, 1, num_tags, GFP_ATOMIC);

	return res;
}

static int uasp_set_alternate(struct usb_interface *iface)
{
	struct usb_device *udev = interface_to_usbdev(iface);
	int i, res;

	for (i = 0; i < iface->num_altsetting; i++) {
		struct usb_host_interface *hi = &iface->altsetting[i];

		if (hi->desc.bInterfaceProtocol == USB_PR_UAS) {
			int ifnum = iface->cur_altsetting->
				desc.bInterfaceNumber;
			int alt = hi->desc.bAlternateSetting;

			res = usb_set_interface(udev, ifnum, alt);
			if (res) {
				dev_err(&udev->dev, "%s: Couldn't "
					"set alternate %d iface (%d)\n",
					__func__, alt, res);
				return -ENODEV;
			}
			return 0;
		}
	}
	dev_err(&udev->dev, "%s: Match, but no suitable alt "
		"iface setting\n", __func__);
	return -ENODEV;
}

/* TODO: We should ideally register a SCSI Target port with the SCSI
 * subsystem and let the SCSI Core perform, in that order, REPORT LUN,
 * INQUIRY, TUR, etc, for each LU, and register each LU as a SCSI
 * Device.
 *
 * We should modify/change/remove the struct scsi_host concept, as we
 * now see SCSI over a myriad of other protocols, where the host
 * controller is NOT a SCSI controller at all.  This however deserves
 * it's own patch.
 */
static int uasp_probe(struct usb_interface *iface,
		      const struct usb_device_id *id)
{
	int res;
	struct Scsi_Host *shost = NULL;
	struct uasp_tport_info *tpinfo;
	struct usb_device *udev = interface_to_usbdev(iface);

	if (id->bInterfaceProtocol == USB_PR_BULK) {
		res = uasp_set_alternate(iface);
		if (res)
			return res;
	}

	tpinfo = kzalloc(sizeof(struct uasp_tport_info), GFP_ATOMIC);
	if (tpinfo == NULL)
		return -ENOMEM;

	tpinfo->iface = iface;
	tpinfo->udev = udev;

	res = uasp_ep_conf(tpinfo);
	if (res)
		goto Free;

	res = -ENOMEM;
	shost = scsi_host_alloc(&uasp_host_template, sizeof(void *));
	if (shost == NULL)
		goto Free;

	/* The protocol supports XCDB, but that won't fit unsigned short,
	 * so claim variable length CDB.
	 */
	shost->max_cmd_len = 260;
	shost->max_id = 1;
	shost->sg_tablesize = udev->bus->sg_tablesize;
	shost->can_queue = tpinfo->max_cmds;
	shost->cmd_per_lun = tpinfo->max_cmds;

	dev_info(&shost->shost_gendev, "max commands: %d\n", tpinfo->max_cmds);

	res = scsi_add_host(shost, &iface->dev);
	if (res)
		goto Free;

	shost->hostdata[0] = (unsigned long)tpinfo;
	usb_set_intfdata(iface, shost);
	scsi_scan_host(shost);

	return res;
 Free:
	kfree(tpinfo);
	if (shost)
		scsi_host_put(shost);
	return res;
}

static int uasp_pre_reset(struct usb_interface *iface)
{
	return 0;
}

static int uasp_post_reset(struct usb_interface *iface)
{
	return 0;
}

static void uasp_disconnect(struct usb_interface *iface)
{
	struct Scsi_Host *shost = usb_get_intfdata(iface);
	struct uasp_tport_info *tpinfo = (void *)shost->hostdata[0];

	scsi_remove_host(shost);
	usb_free_streams(iface, &tpinfo->eps[FIRST_EP_WSTREAMS],
			 NUM_EPS_WSTREAMS, GFP_ATOMIC);
	uasp_free_tags(&tpinfo->tag_data);

	kfree(tpinfo);
}

static struct usb_driver uasp_driver = {
	.name = DRIVER_NAME,
	.probe = uasp_probe,
	.disconnect = uasp_disconnect,
	.pre_reset = uasp_pre_reset,
	.post_reset = uasp_post_reset,
	.id_table = uasp_usb_ids,
};

static int uasp_init(void)
{
	return usb_register(&uasp_driver);
}

static void uasp_exit(void)
{
	usb_deregister(&uasp_driver);
}

module_init(uasp_init);
module_exit(uasp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luben Tuikov");

#ifndef _LINUX_ABUSE_H
#define _LINUX_ABUSE_H

/*
 * include/linux/abuse.h
 *
 * Copyright 2009 by Zachary Amsden.  Redistribution of this file is
 * permitted under the GNU General Public License.
 */

/*
 * Loop flags
 */
enum {
	ABUSE_FLAGS_READ_ONLY	= 1,
	ABUSE_FLAGS_RECONNECT	= 2,
};

#include <linux/types.h>	/* for __u64 */
#include <linux/ioctl.h>

/*
 * ab_size: size of the device in bytes
 * ab_blocksize: block size of the device in bytes
 * ab_max_queue: maximum size of the queue (max number of pending bio's)
 */

struct abuse_info {
	__u64		   ab_device;			/* ioctl r/o */
	__u64		   ab_size;			/* ioctl r/w */
	__u32		   ab_number;			/* ioctl r/o */
	__u32		   ab_flags;			/* ioctl r/w */
	__u32		   ab_blocksize;		/* ioctl r/w */
	__u32		   ab_max_queue;		/* ioctl r/w */
	__u32		   ab_queue_size;		/* ioctl r/o */
	__u32		   ab_errors;			/* ioctl r/o */
	__u32		   ab_max_vecs;			/* ioctl r/o */
};

struct abuse_vec {
	__u64			ab_address;
	__u32			ab_len;
	__u32			ab_offset;
};

struct abuse_xfr_hdr {
	__u64			ab_id;
	__u64			ab_sector;
	__u32			ab_command;
	__u32			ab_result;
	__u32			ab_vec_count;
	__u32			ab_vec_offset;
	__u64			ab_transfer_address;
};

/*
 * ab_command codes
 */
enum {
	ABUSE_READ			= 0,
	ABUSE_WRITE			= 1,
	ABUSE_SYNC_NOTIFICATION		= 2
};

/*
 * ab_result codes 
 */
enum {
	ABUSE_RESULT_OKAY		= 0,
	ABUSE_RESULT_MEDIA_FAILURE	= 1,
	ABUSE_RESULT_DEVICE_FAILURE	= 2
};

/*
 * IOCTL commands
 */

#define ABUSE_GET_STATUS	_IOR('A', 0, struct abuse_info)
#define ABUSE_SET_STATUS	_IOW('A', 1, struct abuse_info)
#define ABUSE_RESET		_IO( 'A', 2)
#define ABUSE_GET_BIO		_IOR('A', 3, struct abuse_xfr_hdr)
#define ABUSE_PUT_BIO		_IOW('A', 4, struct abuse_xfr_hdr)

#endif

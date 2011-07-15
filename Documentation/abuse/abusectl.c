#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <ctype.h>
#include <poll.h>

#include "abuse.h"

#define __unused	__attribute__ ((unused))

/* LT: I chose not to export ABUSE_FLAGS_RECONNECT. If the server
 * wants to reconnect, then it can open the file descriptor, get the
 * info struct and choose not to do a setup.
 */

const char *prog;

typedef enum {
	OP_NONE,
	OP_RESET,
	OP_INFO,
	OP_GET,
	OP_PUT,
	OP_POLL,
	OP_SERVER,
	OP_SETUP,
} operation_t;

struct abuse_opts {
	operation_t	operation;
	const char *dev_name;
	__u64	device_size;
	__u32	block_size;
	__u32	queue_size;
	int	read_only;
};

#define ABUSE_DEF_BLOCK_SIZE	4096
#define ABUSE_DEF_DEV_SIZE	4096*4096LL
#define ABUSE_DEF_Q_SIZE	32

#define ABUSE_DEFAULT_OPTS  {			\
	.operation = OP_NONE,			\
	.dev_name = NULL,			\
	.device_size = ABUSE_DEF_DEV_SIZE,	\
	.block_size = ABUSE_DEF_BLOCK_SIZE,	\
	.queue_size = ABUSE_DEF_Q_SIZE, 	\
	.read_only = 0,				\
}

static int get_number(const int argc, char *argv[], int ii, long long *val)
{
	if (ii >= argc || *argv[ii] == '\0') {
		fprintf(stderr, "%s: %s: missing argument\n", prog,
			argv[ii-1]);
		return -1;
	} else {
		char *ep;

		*val = strtoull(argv[ii], &ep, 0);
		if (ep == argv[ii]) {
			fprintf(stderr, "%s: %s %s: number argument expected\n",
				prog, argv[ii-1], argv[ii]);
			return -1;
		} else if (tolower(*ep) == 'k') {
			*val <<= 10;
			return 0;
		} else if (tolower(*ep) == 'm') {
			*val <<= 20;
			return 0;
		} else if (tolower(*ep) == 'g') {
			*val <<= 30;
			return 0;
		} else if (*ep != '\0') {
			fprintf(stderr, "%s: %s %s: bad suffix\n",
				prog, argv[ii-1], argv[ii]);
			return -1;
		} else {
			return 0;
		}
	}
}

#if 0
static void print_opts(struct abuse_opts *opts)
{
	fprintf(stderr, "%s: op:%d dev:%s size:%lld "
		"bs:%d qs:%d ro:%d\n",
		prog, opts->operation, opts->dev_name,
		opts->device_size, opts->block_size, opts->queue_size,
		opts->read_only);
}
#endif

static int get_opts(const int argc, char *argv[], struct abuse_opts *opts)
{
	int ii, res;
	long long val;

	res = 0;
	for (ii = 1; ii < argc; ii++) {
		if (strcmp("--reset", argv[ii]) == 0) {
			opts->operation = OP_RESET;
		} else if (strcmp("--info", argv[ii]) == 0) {
			opts->operation = OP_INFO;
		} else if (strcmp("--get", argv[ii]) == 0) {
			opts->operation = OP_GET;
		} else if (strcmp("--put", argv[ii]) == 0) {
			opts->operation = OP_PUT;
		} else if (strcmp("--poll", argv[ii]) == 0) {
			opts->operation = OP_POLL;
		} else if (strcmp("--server", argv[ii]) == 0) {
			opts->operation = OP_SERVER;
		} else if (strcmp("--setup", argv[ii]) == 0) {
			opts->operation = OP_SETUP;
		} else if (strcmp("--size", argv[ii]) == 0) {
			res |= get_number(argc, argv, ii+1, &val);
			opts->device_size = val;
			ii += 1;
		} else if (strcmp("--bs", argv[ii]) == 0) {
			res |= get_number(argc, argv, ii+1, &val);
			opts->block_size = val;
			ii += 1;
		} else if (strcmp("--qs", argv[ii]) == 0) {
			res |= get_number(argc, argv, ii+1, &val);
			opts->queue_size = val;
			ii += 1;
		} else if (strcmp("--ro", argv[ii]) == 0) {
			opts->read_only = 1;
		} else {
			opts->dev_name = argv[ii];
		}
	}

	return res;
}

static void usage(struct abuse_opts *opts)
{
	printf("Usage: %s <cmd> [<args>] <dev>\n"
	       "Where <cmd> is one of:\n"
	       "  --reset  - reset the device parameters to 0\n"
	       "  --info   - get info from an abuse device\n"
	       "  --get    - get a bio from an abuse device\n"
	       "  --put    - get and put a bio t/from an abuse device\n"
	       "  --poll   - poll for a bio on device\n"
	       "  --server - act as a server getting and putting bios\n"
	       "  --setup  - setup the device parameters\n"
	       "And <args> is one of:\n"
	       "  --size <N> - device size, suffix [,K,M,G], default: %llu\n"
	       "  --bs <N>   - block size, suffix [,K], default: %d\n"
	       "  --qs <N>   - queue size, max number of bio's pending, "
	       "default: %d\n"
	       "  --ro       - make the device read-only (default: rw)\n",
	       prog,
	       opts->device_size, opts->block_size, opts->queue_size);
	exit(-1);
}

int open_device(const char *fname)
{
	int fd;

	fd = open(fname, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "%s: open %s: %s (%d)\n",
			prog, fname, strerror(errno), errno);
		exit(fd);
	}

	return fd;
}

void do_reset(int fd)
{
	int res;

	res = ioctl(fd, ABUSE_RESET);
	if (res < 0) {
		fprintf(stderr, "%s: ioctl: ABUSE_RESET: %s (%d)\n",
			prog, strerror(errno), errno);
		exit(res);
	}
}

void do_info(int fd, const struct abuse_opts *opts)
{
	int res;
	struct abuse_info ab;

	res = ioctl(fd, ABUSE_GET_STATUS, &ab);
	if (res < 0) {
		fprintf(stderr, "%s: ioctl ABUSE_GET_STATUS: %s (%d)\n",
			prog, strerror(errno), errno);
		exit(res);
	} else {
		printf("%s:\n", opts->dev_name);
		printf("  device size: %lld\n", ab.ab_size);
		printf("  number: %d\n", ab.ab_number);
		printf("  flags: %x\n", ab.ab_flags);
		printf("  blocksize: %d\n", ab.ab_blocksize);
		printf("  max_queue: %d\n", ab.ab_max_queue);
		printf("  queue_size: %d\n", ab.ab_queue_size);
		printf("  errors: %d\n", ab.ab_errors);
		printf("  max_vecs: %d\n", ab.ab_max_vecs);
	}
}

void do_setup(int fd, struct abuse_opts *opts)
{
	int res;
	struct abuse_info ab;

	memset(&ab, 0, sizeof ab);

	ab.ab_size = opts->device_size;
	ab.ab_blocksize = opts->block_size;
	ab.ab_max_queue = opts->queue_size;
	ab.ab_flags = opts->read_only ? ABUSE_FLAGS_READ_ONLY : 0;

	res = ioctl(fd, ABUSE_SET_STATUS, &ab);
	if (res < 0) {
		fprintf(stderr, "%s: ioctl SET STATUS:%s (%d)\n",
			prog, strerror(errno), errno);
		exit(res);
	}
}

static void ab_getbio(int fd, struct abuse_xfr_hdr *hdr,
	       struct abuse_vec *xfr, int count __unused)
{
	int res, ii;

	memset(hdr, 0, sizeof(*hdr));
	hdr->ab_transfer_address = (__u64)xfr;

	res = ioctl(fd, ABUSE_GET_BIO, hdr);
	if (res < 0) {
		fprintf(stderr, "%s: ioctl ABUSE_GET_BIO: %s (%d)\n",
			prog, strerror(errno), errno);
		exit(res);
	}

	printf("%s: sector:%10lld vcnt:%4d ",
	       hdr->ab_command == ABUSE_READ ? "READ" : "WRITE",
	       hdr->ab_sector, hdr->ab_vec_count);

	for (ii = 0; ii < (int) hdr->ab_vec_count; ii++) {
		printf(" [%d]: len:%10u offs:0x%x\n", ii, xfr[ii].ab_len,
		       xfr[ii].ab_offset);
	}
}

void do_getbio(int fd)
{
	struct abuse_xfr_hdr hdr;
	struct abuse_vec xfr[512];

	ab_getbio(fd, &hdr, xfr, 512);
}

void do_putbio(int fd)
{
	int res, ii;
	struct abuse_xfr_hdr hdr;
	struct abuse_vec xfr[512];

	ab_getbio(fd, &hdr, xfr, 512);

	for (ii = 0; ii < (int) hdr.ab_vec_count; ii++) {
		xfr[ii].ab_address = (__u64)malloc(xfr[ii].ab_len);
		if (xfr[ii].ab_address == 0) {
			hdr.ab_result = ABUSE_RESULT_MEDIA_FAILURE;
			break;
		}
	}

	if (hdr.ab_command == ABUSE_WRITE) {
		res = ioctl(fd, ABUSE_GET_WRITE_DATA, &hdr);
		if (res < 0) {
			fprintf(stderr, "%s: ioctl ABUSE_GET_WRITE_DATA: %s "
				"(%d)\n", prog, strerror(errno), errno);
			exit(res);
		}
	}

	res = ioctl(fd, ABUSE_PUT_BIO, &hdr);
	if (res < 0) {
		fprintf(stderr, "%s: ioctl ABUSE_PUT_BIO: %s (%d)\n",
			prog, strerror(errno), errno);
		exit(res);
	}

	for (ii = 0; ii < (int) hdr.ab_vec_count; ii++) {
		free((void *)xfr[ii].ab_address);
	}
}

#ifndef POLLMSG
#define POLLMSG         0x0400
#endif

void do_poll(int fd)
{
	int res;
	struct pollfd fds;

	fds.fd = fd;
	fds.events = POLLMSG;

	res = poll(&fds, 1, -1);
	if (res < 0) {
		fprintf(stderr, "%s: poll: %s (%d)\n", prog, strerror(errno),
			errno);
		exit(res);
	}

	printf("%s: poll: %d revents:0x%x\n", prog, res, fds.revents);
}

int main(int argc, char *argv[])
{
	int fd;
	struct abuse_opts opts = ABUSE_DEFAULT_OPTS;
	int res;

	prog = strrchr(argv[0], '/');
	if (prog == NULL)
		prog = argv[0];
	else
		prog++;

	if (argc < 3)
		usage(&opts);

	res = get_opts(argc, argv, &opts);
	if (res)
		return res;

	if (opts.dev_name == NULL || opts.dev_name[0] == '\0') {
		fprintf(stderr, "%s: no device specified\n", prog);
		return -1;
	}

	fd = open_device(opts.dev_name);

	switch (opts.operation) {
	case OP_RESET:
		do_reset(fd);
		break;
	case OP_INFO:
		do_info(fd, &opts);
		break;
	case OP_GET:
		do_getbio(fd);
		break;
	case OP_PUT:
		do_putbio(fd);
		break;
	case OP_POLL:
		do_poll(fd);
		break;
	case OP_SERVER:
		while (1) {
			do_poll(fd);
			do_putbio(fd);
		}
		break;
	case OP_SETUP:
		do_setup(fd, &opts);
		break;
	default:
		fprintf(stderr, "%s: command argument(s) unknown or "
			"not specified\n", prog);
		break;
	}

	close(fd);
	return 0;
}

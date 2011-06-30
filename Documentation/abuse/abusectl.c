#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <poll.h>
#include "abuse.h"

void usage(void)
{
	printf("abusectl <cmd>\n"
		"	reset <dev>	   - issue a reset on an abuse device\n"
		"	info <dev>	   - get info from an abuse device\n"
		"	get <dev>	   - get bio from an abuse device\n"
		"	put <dev>	   - put bio to an abuse device\n"
		"	poll <dev>	   - poll for a bio on device\n"
		"	server <dev>	   - act as a server putting bios\n"
		"	setup <dev> <args> - setup device parameters\n"
	 	"		<args> is a list of comma delimited keys\n"
		"		sz/size=<N>		set size (K,M.G ok)\n"
		"		bs/blocksize=<N>	set block size\n"
		"		qs/queusize=<N>		set queue size\n"
		"		ro			make device read-only\n"		
		"		rw			make device writeable\n"		
		"		reconnect		do not reset already setup device\n"		
		);
	exit(1);
}

int get_device(char *fname)
{
	int fd;

	fd = open(fname, O_RDONLY);
	if (fd < 0) {
		perror("open");
		exit(1);
	}
	return fd;
}

int do_reset(int fd)
{
	int ret;
	ret = ioctl(fd, ABUSE_RESET);
	if (ret < 0) {
		perror("ioctl");
		exit(1);
	}
}

int do_info(int fd)
{
	int ret;
	struct abuse_info ab;

	ret = ioctl(fd, ABUSE_GET_STATUS, &ab);
	if (ret < 0) {
		perror("ioctl");
		exit(1);
	}
	printf("ab_size = %lld\n", ab.ab_size);
	printf("ab_number = %d\n", ab.ab_number);
	printf("ab_flags = %x\n", ab.ab_flags);
	printf("ab_blocksize = %d\n", ab.ab_blocksize);
	printf("ab_max_queue = %d\n", ab.ab_max_queue);
	printf("ab_queue_size = %d\n", ab.ab_queue_size);
	printf("ab_errors = %d\n", ab.ab_errors);
}

int do_setup(int fd)
{
	int ret;
	struct abuse_info ab;

	memset(&ab, '\0', sizeof(ab));
	ab.ab_size = 4096 * 4096;
	ab.ab_blocksize = 4096;
	ab.ab_max_queue = 128;
	ab.ab_flags = 0;
	ret = ioctl(fd, ABUSE_SET_STATUS, &ab);
	if (ret < 0) {
		perror("ioctl");
		exit(1);
	}
}

int do_getbio(int fd)
{
	int ret, i;
	struct abuse_xfr_hdr hdr;
	struct abuse_vec xfr[512];

	memset(&hdr, '\0', sizeof(hdr));
	hdr.ab_transfer_address = (__u64)xfr;
	ret = ioctl(fd, ABUSE_GET_BIO, &hdr);
	if (ret < 0) {
		perror("ioctl");
		exit(1);
	}
	if (hdr.ab_command == ABUSE_READ)
		printf("READ\n");
	else
		printf("WRITE\n");
	printf("sector = %lld\n", hdr.ab_sector);
	printf("vcnt = %d\n", hdr.ab_vec_count);
	for (i = 0; i < hdr.ab_vec_count; i++) {
		printf("len%d = %lld, offset = %llx\n", i, xfr[i].ab_len,
			xfr[i].ab_offset);
	}
}

int do_putbio(int fd)
{
	int ret, i;
	struct abuse_xfr_hdr hdr;
	struct abuse_vec xfr[512];

	memset(&hdr, '\0', sizeof(hdr));
	hdr.ab_transfer_address = (__u64)xfr;
	ret = ioctl(fd, ABUSE_GET_BIO, &hdr);
	if (ret < 0) {
		perror("ioctl");
		exit(1);
	}
	if (hdr.ab_command == ABUSE_READ)
		printf("READ\n");
	else
		printf("WRITE\n");
	printf("sector = %lld\n", hdr.ab_sector);
	printf("vcnt = %d\n", hdr.ab_vec_count);
	for (i = 0; i < hdr.ab_vec_count; i++) {
		printf("len%d = %lld, offset = %llx\n", i, xfr[i].ab_len,
			xfr[i].ab_offset);
		xfr[i].ab_address = (__u64)malloc(xfr[i].ab_len);
	}
	ret = ioctl(fd, ABUSE_PUT_BIO, &hdr);
	if (ret < 0) {
		perror("ioctl");
		exit(1);
	}
	for (i = 0; i < hdr.ab_vec_count; i++) {
		free((void *)xfr[i].ab_address);
	}
}

#ifndef POLLMSG
#define POLLMSG         0x0400
#endif

int do_poll(int fd)
{
	int ret;
	struct pollfd fds;

	fds.fd = fd;
	fds.events = POLLMSG;
	ret = poll(&fds, 1, -1);
	if (ret < 0) {
		perror("poll");
		exit(1);
	}
	printf("%d %lx\n", ret, fds.revents);
}

int main(int argc, char **argv)
{
	int fd;

	if (argc < 3) {
		usage();
	}
	fd = get_device(argv[2]);
	if (!strcmp(argv[1], "reset")) {
		do_reset(fd);
	} else if (!strcmp(argv[1], "info")) {
		do_info(fd);
	} else if (!strcmp(argv[1], "setup")) {
		do_setup(fd);
	} else if (!strcmp(argv[1], "get")) {
		do_getbio(fd);
	} else if (!strcmp(argv[1], "put")) {
		do_putbio(fd);
	} else if (!strcmp(argv[1], "poll")) {
		do_poll(fd);
	} else if (!strcmp(argv[1], "server")) {
		for (;;) {
			do_poll(fd);
			do_putbio(fd);
		}
	} else usage();
}

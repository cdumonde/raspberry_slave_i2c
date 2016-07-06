#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <curses.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define TX_BUF_SIZE   4000

#define DEFAULT_DEVICE "/dev/i2c_slave"

int main(int argc, char **argv)
{
	char tx_buffer[TX_BUF_SIZE];
	int fd;
	uint8_t data;
	int length;
	int i;

	int opt;
	int mode = 0;
	FILE *usage_file = stderr;
	const char *input = DEFAULT_DEVICE;

	while ((opt = getopt(argc, argv, "hxc")) != -1) {
		switch (opt) {
		case 'd':
			mode = 0; // TODO defines
			break;
		case 'c':
			mode = 1; // TODO defines
			break;
		case 'x':
			mode = 2; // TODO defines
			break;
		case 'h':
			usage_file = stdout;
		default: /* '?' */
			fprintf(usage_file, "Usage: %s [-a addr] [-x|-c|-d] [device]\n", argv[0]);
			fprintf(usage_file, "defaults device: %s addr: %x\n", DEFAULT_DEVICE, 0x42);
			exit(usage_file == stdout ? EXIT_SUCCESS : EXIT_FAILURE);
		}
	}

	if (optind < argc) {
		input = argv[optind];
	}

	if ((fd = open(input, O_RDWR)) == -1) {
		perror("open i2c device");
		exit(EXIT_FAILURE);
	}

	while (1) {	
			length = read(fd, tx_buffer, TX_BUF_SIZE);
			for(i = 0; i < length; i++)
			{
				switch (mode) {
				case 1:
					printf("Data received : %c\n", tx_buffer[i]);
					break;
				case 2:
					printf("Data received : %02x\n ", tx_buffer[i]);
					break;
				default:
					printf("Data received : %d \n", tx_buffer[i]);
					break;
				}
			}
			write(fd, tx_buffer, length);
	}

	close(fd);
	return 0;
}

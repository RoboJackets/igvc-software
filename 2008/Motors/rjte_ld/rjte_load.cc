#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <iostream>
#include <string>

#include "serial.h"
#include "exception.h"

using namespace std;

#define PAGE_SIZE	64

extern const char *port;

void handshake()
{
	int state = 0;

	/* Assert DTR so that the bootloader will start the handshake procedure.
	 *
	 * Hopefully the user code will detect this and reset so the bootloader will
	 * run without user intervention.
	 */
	ser_dtr(1);

	while (1)
	{
		unsigned char ch = ser_receive();

		/* Wait for "T1" */
		if (state == 0 && ch == 'T')
			state = 1;
		else if (state == 1 && ch == '1')
			break;
	}

	/* Tell the target that we are about to start sending data */
	ser_send('H');
	ser_send(1);
}

void send_page(unsigned char *data)
{
	int i, ch, cl, good_sum, target_sum;

	good_sum = 0;
	for (i = 0; i < PAGE_SIZE; i += 2)
	{
		ser_send(data[i]);
		ser_send(data[i + 1]);

		good_sum += (data[i + 1] << 8) | data[i];
	}
	good_sum &= 0xffff;

	cl = ser_receive_timed();
	ch = ser_receive_timed();
	target_sum = (ch << 8) | cl;

	if (target_sum != good_sum)
		throw text_exception("Checksum failed");
}

void send_file(FILE *fp)
{
	unsigned int len, total_len, last_frac;
	unsigned char page[PAGE_SIZE];

	/* Get the length of the file */
	fseek(fp, 0, SEEK_END);
	total_len = len = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	printf("|");
	for (int i = 0; i < 50; i++)
		printf("-");
	printf("|\r|");
	fflush(stdout);

	/* Send whole pages */
	last_frac = 0;
	for (; len >= 64; len -= PAGE_SIZE)
	{
		if (fread(page, 1, PAGE_SIZE, fp) != PAGE_SIZE)
			throw errno_exception("Can't read from input file");

		send_page(page);

		unsigned int frac = 50 * (total_len - len) / total_len;
		for (unsigned int i = last_frac; i < frac; i++)
			printf("*");
		fflush(stdout);
		last_frac = frac;
	}

	/* Send the last partial page */
	if (len)
	{
		memset(page, 0xff, PAGE_SIZE);
		if (fread(page, 1, len, fp) != len)
			throw errno_exception("Can't read from input file");

		send_page(page);

		for (unsigned int i = last_frac; i < 50; i++)
			printf("*");
		fflush(stdout);
	}

	printf("\n");
}

int main(int argc, char *argv[])
{
	if ((argc < 2) || (argc > 4))
	{
		fprintf(stderr, "Usage: rjte_load <filename.bin> [port]\n");
		return 1;
	}

	FILE *fp = fopen(argv[1], "rb");
	if (!fp)
	{
		fprintf(stderr, "Can't open %s: %s\n", argv[1], strerror(errno));
		return 1;
	}
	port = ( (argc == 3) ? argv[3] : DEFAULT_PORT );


	try
	{
		ser_init();

		cout << "Reset the board..." << endl;
		handshake();

		send_file(fp);

		// Indicate the end of the data
		ser_dtr(0);
	} catch (exception &err)
	{
		cout.flush();
		cerr << endl << err.what() << endl;

		ser_shutdown();
		return 1;
	}

	ser_shutdown();

	return 0;
}


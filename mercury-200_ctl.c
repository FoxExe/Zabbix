#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>

//#define TTY_DEV_PATH "/dev/ttyUSB0"
//#define TTY_DEV_PATH "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
#define TTY_DEV_SPEED B9600

int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);	// Why? idk...

    if (tcgetattr(fd, &tty) < 0) {
        printf("ZBX_ERR_%s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);	// shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);		// ignore modem controls,
	tty.c_cflag &= ~(PARENB | PARODD);		// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_iflag &= ~IGNBRK;         // ignore break signal
	tty.c_lflag = 0;                // no signaling chars, no echo,
	tty.c_oflag = 0;                // no remapping, no delays

	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("ZBX_ERR_%s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("ZBX_ERR_SET_BLOCKING\n");
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf ("ZBX_ERR_SET_BLOCKING_2\n");
}

uint16_t CRC16 (const uint8_t *nData, uint16_t wLength)
{
    static const uint16_t crcTable[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

    uint8_t nTemp;
    uint16_t crcSum = 0xFFFF;

    while (wLength--)
    {
        nTemp = *nData++ ^ crcSum;
        crcSum >>= 8;
        crcSum  ^= crcTable[(nTemp & 0xFF)];
    }
    return crcSum;
}

int main(int argc, char** argv) {
	// TODO: Add key --silent (Only parsed output)
	if (argc != 4) {
		printf("ZBX_ERR_NOT_ALL_ARGS:\n");
		printf("%s <SN_ID> <CMD_ID>\n", argv[0]);
		return 1;
	}

	int dev_id = atoi(argv[2]);
	if (dev_id < 0 || dev_id > 999999) {
		printf("ZBX_WRONG_ID\n");
		return 2;
	}

	uint8_t cmd_id = (uint8_t)atoi(argv[3]);
	if (cmd_id < 0 || cmd_id > 255) {
		printf("ZBX_WRONG_CMD\n");
		return 2;
	}

	uint8_t cmd_send[7];
	cmd_send[0] = (dev_id >> 24) & 0xFF;
	cmd_send[1] = (dev_id >> 16) & 0xFF;
	cmd_send[2] = (dev_id >> 8) & 0xFF;
	cmd_send[3] = dev_id & 0xFF;
	cmd_send[4] = cmd_id;
	cmd_send[5] = 0;
	cmd_send[6] = 0;

	uint16_t cmd_crc = CRC16(cmd_send, 5);
	cmd_send[5] = 0x00FF & cmd_crc;
	cmd_send[6] = (0xFF00 & cmd_crc) >> 8;

/*
	// Debug
	printf("CMD: %02X %02X %02X %02X %02X %02X %02X\n",
		cmd_send[0],
		cmd_send[1],
		cmd_send[2],
		cmd_send[3],
		cmd_send[4],
		cmd_send[5],
		cmd_send[6]);
*/

// =============== RS485 request ===============

	int tty_fd;
	struct termios oldtty_io;
	int dataLenght = 0;

	//Save terminal configuratty_ion
	tcgetattr(STDOUT_FILENO, &oldtty_io);
	tty_fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
	if (tty_fd < 0) {
		printf("ZBX_ERR_NO_ACCESS_%s:_%s\n", argv[1], strerror(errno));
		return -1;
	}

	set_interface_attribs(tty_fd, TTY_DEV_SPEED, 0);
	set_blocking (tty_fd, 1);

	dataLenght = write(tty_fd, &cmd_send, 7);
	usleep ((7 + 25) * 100);

	if (dataLenght < 0) {
		printf("ZBX_CMD_SEND_ERROR%d\n", dataLenght);
		return -1;
	}

	// Change buf lenght to available_bytes_for_read
	dataLenght = 0;	// reset (Not needed, but who care?)
	uint8_t buf[128];
	dataLenght = read(tty_fd, &buf, sizeof(buf));

	if (dataLenght < 0) {
		printf("ZBX_CMD_READ_ERROR_%d\n", dataLenght);
		return -1;
	} else if (dataLenght > 8) {	// Minimu is 8 bytes (SN+CMD+ANSWER+CRC)
		// Check CRC
		if (CRC16(buf, dataLenght - 2) != ((buf[dataLenght - 1] << 8) | buf[dataLenght - 2])) {
			printf("ZBX_CMD_CRC_ERROR\n");
			return -2;
		}

		switch (cmd_id) {
			// Calculated Watt/h per tariff (4 tariffs)
			case 0x27:
				; // Fix "Empty statement" error

				// 4 byte SN, 1 byte CMD, 4x4 byte data, 2 byte CRC = 23 total
				// Data string: 123456.78
				char float_wh[9];
				sprintf(float_wh, "%02x%02x%02x.%02x", buf[5], buf[6], buf[7], buf[8]);
				printf("%.02f ", strtof(float_wh, NULL));
				sprintf(float_wh, "%02x%02x%02x.%02x", buf[9], buf[10], buf[11], buf[12]);
				printf("%.02f ", strtof(float_wh, NULL));
				sprintf(float_wh, "%02x%02x%02x.%02x", buf[13], buf[14], buf[15], buf[16]);
				printf("%.02f ", strtof(float_wh, NULL));
				sprintf(float_wh, "%02x%02x%02x.%02x", buf[17], buf[18], buf[19], buf[20]);
				printf("%.02f\n", strtof(float_wh, NULL));
			break;

			// Current power data
			case 0x63:
				; // Fix "Empty statement" error
				// 3x2 = 6 bytes of data
				char text_current[5];
				sprintf(text_current, "%02x%02x", buf[5], buf[6]);
				printf("%.02f ", strtof(text_current, NULL) / 10);
				sprintf(text_current, "%02x.%02x", buf[7], buf[8]);
				printf("%.02f\n", strtof(text_current, NULL));
			break;

			default:
				printf("ZBX_UNC");
				for (int i = 0; i < dataLenght; i++) {
					printf("_%02X", buf[i]);
				}
				printf("\n");
			break;
		}
	}

	close(tty_fd);

	//back to initial terminal configuratty_ion
	tcsetattr(STDOUT_FILENO, TCSANOW, &oldtty_io);
	tcsetattr(STDOUT_FILENO, TCSAFLUSH, &oldtty_io);

	return 0;
}
